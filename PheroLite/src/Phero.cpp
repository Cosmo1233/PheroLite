#include <Arduino.h>
#include "phero.hpp"

Phero_c::Phero_c(int cell_width, int start_x, int start_y, 
                 float evap_rate, uint8_t r_phero_val, uint8_t phero_type_val)
    : cell_w(cell_width), pmap_start_x(start_x), pmap_start_y(start_y),
      evaporation(evap_rate), r_phero(r_phero_val), phero_type(phero_type_val) {
    }

void Phero_c::initialise(void) {
    // Clear pheromone map
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            pmap[i][j] = 0.0f;
        }
    }
  
    // Reset position to starting point
    goalcell_x = pmap_start_x;
    goalcell_y = pmap_start_y;
    pmap_x = pmap_start_x;
    pmap_y = pmap_start_y;
    prev_pmap_x = pmap_start_x;
    prev_pmap_y = pmap_start_y;
    fwdX_global = pmap_x; 
    fwdY_global = pmap_y + 1;
    
    // Reset mission state
    message_type = 0;
    n_rand_cells = 5;
    message_index_hybrid = 0;
    initialised = false;
}


void Phero_c::update(float robot_x, float robot_y, float robot_theta) {
  updatePmapxy(robot_x,  robot_y, robot_theta); 
  detection();
  deposition();
  initialised = true;

}

// Deposition of pheromone
void Phero_c::deposition() {
  // out of bounds, no phero trail or incorrect option, do nothing
  if (pmap_x < 0 || pmap_x >= MAP_SIZE || pmap_y < 0 || pmap_y >= MAP_SIZE
      || phero_type <= 0 || phero_type > 2) {
    return;
  }

  for (int y = 0; y < MAP_SIZE; y++) {
    for (int x = 0; x < MAP_SIZE; x++) {

      uint8_t max_phero, min_phero;
      int8_t radius = (int)max(abs(y - pmap_y), abs(x - pmap_x)); // Manhattan distance

      // If cell is within robot's view radius (also changes neighbour pheromone to own if in radius and laying pheromone)
      if (radius <= r_phero) {
        double diffusion = ALPHA * pow((0.1 * exp(1)), (2 * (radius / PI)));

        max_phero = phero_type;
        min_phero = max_phero - 1;

        if (pmap[y][x] < min_phero) pmap[y][x] = min_phero; // prevents local pheromone values from going below range

        pmap[y][x] += (max_phero - pmap[y][x]) * diffusion; // deposition of pheromone within radius

        if (pmap[y][x] > max_phero) pmap[y][x] = max_phero; // prevents value from exceeding relative maximum
      }


      if (pmap[y][x] > 0) {
        if (pmap[y][x] <= 1) {
          pmap[y][x] -= pmap[y][x] * evaporation;
        } else { // Use decimal component for other ranges greater than 1
          float aux = fmod(pmap[y][x], 1.0f);
          pmap[y][x] -= aux * evaporation;
        }

        // below to reset neighbour pheromone ranges when below lower limit:
        if (pmap[y][x] > 1 && pmap[y][x] < 2) pmap[y][x] = 0; // reset local attract phero
        if (pmap[y][x] > 9 && pmap[y][x] < 10) pmap[y][x] = 0; // reset neighbour repulse phero
        if (pmap[y][x] > 11 && pmap[y][x] < 12) pmap[y][x] = 0; // reset neighbour attract phero

      }
    }
  }
}

// Detection of local pheromones used for movement (repulsion or attraction to trails)
void Phero_c::detection(void) {    
    const int SAFETY_MARGIN = 2;  // Cells from edge to trigger boundary avoidance
    
    // Check if robot is near boundaries - override with center goal
    if (pmap_x <= SAFETY_MARGIN || pmap_x >= MAP_SIZE - SAFETY_MARGIN ||
        pmap_y <= SAFETY_MARGIN || pmap_y >= MAP_SIZE - SAFETY_MARGIN) {
        
        // Force goal toward map center to keep robot in bounds
        goalcell_x = MAP_SIZE / 2;
        goalcell_y = MAP_SIZE / 2;
        return;  // Skip normal pheromone detection
    }
    
    // Normal pheromone-based detection
    int8_t best_x = pmap_x, best_y = pmap_y; // Default to current position
    float best_score = -1000; // Initialise to very low value
    
    // Check 8 neighboring cells (3x3 minus center)
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue; // Skip center cell
            
            int8_t x = pmap_x + dx;
            int8_t y = pmap_y + dy;
            
            // Bounds check
            if (x < 0 || x >= MAP_SIZE || y < 0 || y >= MAP_SIZE) continue;
            
            float cell_value = pmap[y][x];
            if (cell_value == 0) continue; // Skip empty cells
            
            // Decode pheromone info
            float concentration = fmod(cell_value, 1.0f);
            bool is_attractive = (cell_value >= 2.0f && cell_value <= 3.0f) || 
                               (cell_value >= 12.0f && cell_value <= 13.0f);
            bool is_repulsive = (cell_value >= 0.0f && cell_value <= 1.0f) || 
                              (cell_value >= 10.0f && cell_value <= 11.0f);
            
            float score = calculateCellScore(dx, dy, concentration, is_attractive, is_repulsive);
            
            if (score > best_score) {
                best_score = score;
                best_x = x;
                best_y = y;
            }
        }
    }
    
    goalcell_x = best_x;
    goalcell_y = best_y;
}


float Phero_c::calculateCellScore(int dx, int dy, float concentration, 
                                 bool is_attractive, bool is_repulsive) {
    float score = 0;  
    // Angle preference (prefer forward direction)
    float angle_penalty = angleDis(dx, fwdX_local, dy, fwdY_local) / 180.0f; // 0-1 scale
    
    // Priority system: Attractive > Empty > Repulsive
    if (is_attractive) {
        // SEEK attractive cells - higher concentration = better target
        score = 200 + (concentration * 100); // Base score 200-300
    } else if (is_repulsive) {
        // AVOID repulsive cells - lower concentration = less bad
        score = -100 + ((1.0f - concentration) * 50); // Score -100 to -50
    } else {
        // Empty cells are neutral
        score = 50; // Better than repulsive, worse than attractive
    }
    
    // Apply directional preference (prefer moving forward)
    score -= angle_penalty * 20;
    return score;
}

int Phero_c::angleDis(int x2, int x1, int y2, int y1) { // 3x3 neighbour cell input -> degree output
  if (x1 < -1 || x1 >= 2 || x2 < -1 || x2 >= 2 || y1 < -1 || y1 >= 2 || y2 < -1 || y2 >= 2) {
    // incorrect input to angleDis;
    return 0;
  }
  int8_t angle_tgt = angles[y2 + 1][x2 + 1]; // +1 to put in range [0,1,2] for array
  int8_t angle_fwd = angles[y1 + 1][x1 + 1];
  int8_t angle_turn = 0;

  if (angle_tgt > angle_fwd) {
    if (angle_tgt - angle_fwd <= 180) {
      angle_turn = angle_tgt - angle_fwd;
    } else {
      angle_turn = 360 - (angle_tgt - angle_fwd);
    }
  } else {
    if (angle_fwd - angle_tgt <= 180) {
      angle_turn = angle_fwd - angle_tgt;
    } else {
      angle_turn = 360 - (angle_fwd - angle_tgt);
    }
  }
  return angle_turn;
}

void Phero_c::ThetaRange360(float robot_theta) { // Gives output in degrees in 0 - 360 range 
  float temp = fmod(robot_theta, 2 * PI);
  if (temp < 0) {
    temp += 2 * PI;
  }
  theta_range =  temp * 180.0 / PI; 
}


float Phero_c::AngleToCenter(void) { // degree output -pi to pi range 
  // Calculate the center of the map
  int center_x = floor((MAP_SIZE - 1) / 2);
  int center_y = floor((MAP_SIZE - 1) / 2);

  // Calculate the angle towards the center
  float dx = center_x - pmap_x;
  float dy = center_y - pmap_y;
  return (atan2(dy, dx) * 180.0 / PI);
}

// updates pmap co-ordinates, theta_range & fwd cells
void Phero_c::updatePmapxy(float robot_x, float robot_y, float robot_theta) {
  prev_pmap_x = pmap_x;
  prev_pmap_y = pmap_y;
  pmap_x = pmap_start_x + (int)(robot_x / cell_w);
  pmap_y = pmap_start_y + (int)(robot_y / cell_w);

  /* updating fwd cell based on theta - assuming orientation is forward at start
        - ^ -
        - X -
        - - -
  */
  ThetaRange360(robot_theta); // updates the normalised theta_range variable

  uint8_t cell_fwdX = 0;
  uint8_t cell_fwdY = 0;
  if (theta_range >= 337.5 || theta_range < 22.5) { // Top (0 degrees)
    cell_fwdX = 0;
    cell_fwdY = 1;
  } else if (theta_range >= 22.5 && theta_range < 67.5) { // Top-right (45 degrees)
    cell_fwdX = 1;
    cell_fwdY = 1;
  } else if (theta_range >= 67.5 && theta_range < 112.5) {   // Right (90 degrees)
    cell_fwdX = 1;
    cell_fwdY = 0;
  } else if (theta_range >= 112.5 && theta_range < 157.5) {  // Bottom-right (135 degrees)
    cell_fwdX = 1;
    cell_fwdY = -1;
  } else if (theta_range >= 157.5 && theta_range < 202.5) { // Bottom (180 degrees)
    cell_fwdX = 0;
    cell_fwdY = -1;
  } else if (theta_range >= 202.5 && theta_range < 247.5) { // Bottom-left (225 degrees)
    cell_fwdX = -1;
    cell_fwdY = -1;
  } else if (theta_range >= 247.5 && theta_range < 292.5) {   // Left (270 degrees)
    cell_fwdX = -1;
    cell_fwdY = 0;
  } else if (theta_range >= 292.5 && theta_range < 337.5) { // Top-left (315 degrees)
    cell_fwdX = -1;
    cell_fwdY = 1;
  }

  fwdX_local = cell_fwdX;
  fwdY_local = cell_fwdY;
  fwdX_global = pmap_x + cell_fwdX;
  fwdY_global = pmap_y + cell_fwdY;
}

// converts decimal into a two digit integer
uint8_t Phero_c::encode_decimal(float value) {
  if (value >= 0 && value <= 1) { 
    return (uint8_t)(value * 100);  // convert to 2 digit uint
  } else {
    return 0; // Error handling for out of range values
  }
}
// converts two digit integer into a decimal
float Phero_c::decode_decimal(uint8_t value) {
  if (value >= 0 && value <= 99) {
    return (value / 100);
  } else {
    return 0;
  }
}

void Phero_c::addRandomCells(uint8_t& msg_index, uint8_t count) {
    for (int i = 0; i < count; i++) {
        int ranX, ranY;
        float ran_pvalue;
        do {
            ranX = random(0, MAP_SIZE);
            ranY = random(0, MAP_SIZE);
            ran_pvalue = fmod(pmap[ranY][ranX], 1.0);
        } while (ran_pvalue == 0);
        
        p_msg[msg_index++] = (pmap[ranY][ranX] < 1 || (pmap[ranY][ranX] >= 10 && pmap[ranY][ranX] <= 11)) ? 'A' : 'B';
        p_msg[msg_index++] = ranX;
        p_msg[msg_index++] = ranY;
        p_msg[msg_index++] = encode_decimal(ran_pvalue);
    }
}

void Phero_c::addLocalCells(uint8_t& msg_index, uint8_t grid_radius_) {
    for (int i = -grid_radius_; i <= grid_radius_; i++) {
        for (int j = -grid_radius_; j <= grid_radius_; j++) {
            uint8_t x = pmap_x + j, y = pmap_y + i;
            p_msg[msg_index++] = encode_decimal(fmod(pmap[y][x], 1.0));
        }
    }
}

void Phero_c::addIDCoordCells(uint8_t& msg_index, char msgID, uint8_t phero_type_, uint8_t pmap_x_, uint8_t pmap_y_) {
    p_msg[msg_index++] = msgID;
    p_msg[msg_index++] = phero_type_;
    p_msg[msg_index++] = pmap_x_;
    p_msg[msg_index++] = pmap_y_;
}

/* 
  msg type 0 (local 3x3 grid) = 'A' + phero type (no trail - 0, repulse - 1 OR attract - 2) (1 byte) + co-ords (2 bytes) + 3x3 grid of values (9 bytes)
  msg type 1 (local 5x5 grid) = 'B' + same as above but with 25 cells
  msg type 2 (random) = 'C' (indicate random cells) +  N random cells * (phero type (1 byte) + co-ords (2 bytes) + phero concentration (1 byte))
  msg type 3 (hybrid sequential) = sends type 0 then 2 sequentially
  msg type 4 (hybrid combination) =  'D' + sends type 0 and 2 at the same time
*/

void Phero_c::msgUpdate(void) {
    memset(p_msg, 0, sizeof(p_msg)); // Clear entire p_msg buffer
    uint8_t msg_index = 0;

    if (message_type == 0 || message_type == 1 || (message_type == 3 && message_index_hybrid == 0)) {
        uint8_t grid_radius = message_type == 1 ? 2 : 1;
        char msgID = message_type == 1 ? 'B' : 'A';  // 'A' for 3x3, 'B' for 5x5
        addIDCoordCells(msg_index, msgID, phero_type, pmap_x, pmap_y);
        addLocalCells(msg_index, grid_radius);
    }
    // Sending random cells
    else if (message_type == 2 || (message_type == 3 && message_index_hybrid == 1)) {
        p_msg[msg_index++] = 'C';  // 'C' for random
        addRandomCells(msg_index, n_rand_cells);
    }
    // Sending Hybrid combination
    else if (message_type == 4) {
        addIDCoordCells(msg_index, 'D', phero_type, pmap_x, pmap_y);  // 'D' for hybrid
        addLocalCells(msg_index, 1);  // 3x3 grid for hybrid
        addRandomCells(msg_index, 3);
    }

    if (message_type == 3) {
        message_index_hybrid = 1 - message_index_hybrid;
    }
}

// Unpack incoming pheromone message
void Phero_c::msgDecode(char new_msg[]) {
    char msgType = new_msg[0];
    if (msgType == 'A' || msgType == 'B') { // local 3x3 or 5x5 grid
        processLocalMessage(new_msg);
    } else if (msgType == 'C') { // random cells
        processRandomCellsMessage(new_msg, 1, n_rand_cells);
    } else if (msgType == 'D') { // hybrid combination format
        processLocalMessage(new_msg);  // Process local part (starts at index 0)
        // Calculate where random cells start: 1 + phero_type + x + y + (3x3 grid = 9 cells) = 13
        processRandomCellsMessage(new_msg, 13, 3);  // Process 3 random cells
    }
}

void Phero_c::processLocalMessage(char* msg) {
    bool local_5 = msg[0] == 'B';  // 'B' indicates 5x5 grid
    
    // Message structure: [ID][phero_type][x][y][grid_data...]
    char phero_msg_type = msg[1];  // Always at index 1
    uint8_t x_coord = msg[2];          // Always at index 2  
    uint8_t y_coord = msg[3];          // Always at index 3

    if (local_5) {
        // 5x5 grid: data starts at index 4
        for (int8_t i = -2, index = 4; i <= 2; i++) {
            for (int8_t j = -2; j <= 2; j++, index++) {
                msgUpdatePmap(x_coord + j, y_coord + i, phero_msg_type, msg[index]);
            }
        }
    } else { 
        // 3x3 grid: data starts at index 4
        for (int8_t i = -1, index = 4; i <= 1; i++) {
            for (int8_t j = -1; j <= 1; j++, index++) {
                msgUpdatePmap(x_coord + j, y_coord + i, phero_msg_type, msg[index]);
            }
        }
    }
}

void Phero_c::processRandomCellsMessage(char* msg, int startIndex, int cellCount) {
    for (int i = 0; i < cellCount; i++) {
        int index = startIndex + (4 * i);
        char ran_phero_type = msg[index];
        int ranX = msg[index + 1];
        int ranY = msg[index + 2];
        msgUpdatePmap(ranX, ranY, ran_phero_type, msg[index + 3]);
    }
}

void Phero_c::msgUpdatePmap(uint8_t x, uint8_t y, uint8_t phero_type, uint8_t value) {
  if (x < MAP_SIZE && y < MAP_SIZE) {
      float auxFlt = decode_decimal(value); 
      float current_value = fmod(pmap[y][x], 1.0);  // extract concentration from current encoded value

      // update if incoming concentration is higher
      if (auxFlt > current_value) {
          pmap[y][x] = auxFlt + (phero_type == 1 ? 10 : 12);
      }
  }
}

// set angle demand based on goal and pmap cells - output in degrees
float Phero_c::ThetaDemand(void) {
    // Calculate direction to goal cell
    int8_t dx = goalcell_x - pmap_x;
    int8_t dy = goalcell_y - pmap_y;
    
    // Calculate angle in radians (-π to π)
    angle_demand = atan2(dy, dx);
    
    return angle_demand;  // Return in radians for flight controller
}

// Random Gaussian used for generating theta demand - used for random walk
float Phero_c::randGaussian( float mean, float sd ) {
  float x1, x2, w, y;
  do {
    // Adaptation here because arduino random() returns a long
    x1 = random(0, 2000) - 1000;
    x1 *= 0.001;
    x2 = random(0, 2000) - 1000;
    x2 *= 0.001;
    w = (x1 * x1) + (x2 * x2);

  } while ( w >= 1.0 );

  w = sqrt( (-2.0 * log( w ) ) / w );
  y = x1 * w;

  return mean + y * sd;
}
