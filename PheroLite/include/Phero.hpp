#ifndef PHERO_HPP
#define PHERO_HPP

#include <Arduino.h>
#include <Wire.h>

/*
 * Phero_c - Virtual Pheromone Class
 * 
 * This class implements a virtual pheromone-based navigation system. 
 * Robots deposit virtual pheromones in a discretised 2D grid map and
 * use local pheromone concentrations to make movement decisions.
 * 
 * Key Features:
 * - Customisable pheromone map size and cell size to a maximum of 255 x 255 cells
 * - Two pheromone types: repulsive (avoid) and attractive (follow)
 * - Distinction between own and neighbour pheromone trails in map
 * - ESP-NOW compatible messaging, with smallest size being 12 bytes
 * - Boundary avoidance to keep robots within map area
 * - Configurable deposition radius and evaporation rates
 * - Various helper functions for changing parameters
 * 
 * Usage:
 *   Phero_c phero(20, 25, 25, 0.036, 1, 1);  // 20mm cells, pmap_x start, pmap_y start, evaporation, pheromone radius, pheromone type
 *   phero.initialise();
 *   phero.update(robot_x, robot_y, robot_theta); // update with robot's own pose
 *   float heading = phero.ThetaDemand();  // Get desired heading
 *   phero.msgUpdate();  // Prepare message for transmission
 * 
 *  // Example method of printing out pmap in standard x-y graph format using Serial.print(y increases upward)
    void printPmapGraph() {
    // Print from top row to bottom (y decreases)
        for (int y = phero.getMapSize() - 1; y >= 0; y--) {
           for (int x = 0; x < phero.getMapSize(); x++) {
                float value = phero.getPmapCell(x, y);
                Serial.printf("%6.2f ", value);  // 6 chars wide, 2 decimal places
            }
        Serial.println();
        }
    }
 *
 */

class Phero_c {
private:
    // Default configuration values - Change below for each use case
    static constexpr uint8_t MAP_SIZE = 50;  // Diameter of map
    static constexpr int CELL_W = 20;        // mm per cell 
    static constexpr int START_X = MAP_SIZE/2;  // center start position
    static constexpr int START_Y = MAP_SIZE/2;
    
    // Default algorithm parameters
    static constexpr float DEFAULT_EVAPORATION = 0.0366598f;
    static constexpr uint8_t DEFAULT_R_PHERO = 1;  
    static constexpr uint8_t DEFAULT_PHERO_TYPE = 1;    
    static constexpr uint8_t MAX_BUFFER = 120;          

    static constexpr float ALPHA = 0.5f;
    
    // Direction lookup table for movement calculations
    const uint16_t angles[3][3] = {
        {225, 270, 315},
        {180, 0, 0},
        {135, 90, 45}
    };
    
    // Internal state - not directly accessible
    uint8_t phero_type;                  // Current encoded pheromone trail -> 0 = no trail, 1 = repulse, 2 = attract
    float pmap[MAP_SIZE][MAP_SIZE];     /* 2D pheromone map containing integer part (phero_type) + decimal (concentration)
                                          y ^
                                            |
                                            |
                                             - - -> x 
                                         */
    
    // Position tracking - managed internally
    int8_t pmap_x;                       // Current grid X coordinate
    int8_t pmap_y;                       // Current grid Y coordinate
    int8_t prev_pmap_x;                  // Previous grid X coordinate
    int8_t prev_pmap_y;                  // Previous grid Y coordinate
    
    // Movement calculation variables - internal use only
    uint8_t fwdX_local;                   // Forward cell X in local 3x3 neighborhood (-1,0,1)
    uint8_t fwdY_local;                   // Forward cell Y in local 3x3 neighborhood (-1,0,1)
    int8_t fwdX_global;                     // Forward cell X in global coordinates
    int8_t fwdY_global;                     // Forward cell Y in global coordinates
    
    // Communication internals
    int message_index_hybrid;            // Index for hybrid sequential messaging
    uint8_t p_msg[MAX_BUFFER];           // Pheromone message buffer 
    
    // Algorithm internals
    float angle_demand;                  // Demanded angle for movement
    float theta_range;                   // Normalised orientation 0 - 360 degrees

public:
    // Configuration parameters (set via constructor)
    const int cell_w;                    // Width of each cell in mm
    const int pmap_start_x;              // Starting X position
    const int pmap_start_y;              // Starting Y position
    
    // Tunable algorithm parameters - accessible for configuration
    float evaporation;                   // Evaporation rate (0.0-1.0)
    uint8_t r_phero;                     // Radius of pheromone deposition e.g. r_phero = 1 -> 8 neighbour cell deposition
    uint8_t message_type;                // Communication strategy (0-4)
    uint8_t n_rand_cells;                // Number of random cells in messages
    
    // Goal/target information - external code needs to set these
    int8_t goalcell_x;                   // Target X coordinate
    int8_t goalcell_y;                   // Target Y coordinate
    
    // Current state information - external code needs to read these
    bool initialised;                    // Initialization status
    
    // Constructors
    Phero_c(int cell_width = CELL_W, 
            int start_x = START_X, 
            int start_y = START_Y,
            float evap_rate = DEFAULT_EVAPORATION,
            uint8_t r_phero = DEFAULT_R_PHERO,
            uint8_t phero_type = DEFAULT_PHERO_TYPE);
    
    // Core algorithm functions
    void initialise(void);
    void update(float robot_x, float robot_y, float robot_theta);   // robot's actual x, y, theta from odometry
    void deposition(void);
    void detection(void);
    void updatePmapxy(float robot_x, float robot_y, float robot_theta);
    
    // Movement and navigation
    void ThetaRange360(float robot_theta);
    float ThetaDemand(void);
    float AngleToCenter(void);
    int angleDis(int x2, int x1, int y2, int y1);
    
    // Communication and messaging
    void msgUpdate(void);
    void msgDecode(char new_msg[]);
    void processLocalMessage(char* msg);
    void processRandomCellsMessage(char* msg, int startIndex, int cellCount);
    void msgUpdatePmap(uint8_t x, uint8_t y, uint8_t phero_type, uint8_t value);
    void addRandomCells(uint8_t& msg_index, uint8_t count);
    void addLocalCells(uint8_t& msg_index, uint8_t grid_radius_ = 1);
    void addIDCoordCells(uint8_t& msg_index, char msgID, uint8_t phero_type_, uint8_t pmap_x_, uint8_t pmap_y_);

    // Encoding/decoding for pheromone concentration compression
    uint8_t encode_decimal(float value);
    float decode_decimal(uint8_t value);
    
    // Utility functions
    float calculateCellScore(int dx, int dy, float concentration, 
                                 bool is_attractive, bool is_repulsive);
    float randGaussian(float mean, float sd);
    void changePhero(uint8_t option) { 
        if(option >= 0 && option <= 2) phero_type = option;
    }
    void changeMsgType(uint8_t option) {
        if(option >= 0 && option <= 4) message_type = option;
    }
    void changeN_Rand_Cells(uint8_t option) {
        if (option >= 0 && option <= 4) n_rand_cells = option;
    }
    
    // Public getters for accessing private state (telemetry/debugging)
    int8_t getPmapX(void) const { return pmap_x; }
    int8_t getPmapY(void) const { return pmap_y; }
    int8_t getPreviousPmapX(void) const { return prev_pmap_x; }
    int8_t getPreviousPmapY(void) const { return prev_pmap_y; }
    uint8_t getMapSize() const { return MAP_SIZE; }
    float getPmapCell(uint8_t x, uint8_t y) { return pmap[y][x]; }
    int getForwardCellX(void) const { return fwdX_global; }
    int getForwardCellY(void) const { return fwdY_global; }
    float getAngleDemand(void) const { return angle_demand; }


    struct PositionData {
        int8_t current_x, current_y;
        int8_t previous_x, previous_y;
    };

    PositionData getPmapXY(void) const {                      // usage: auto pmap_pos_data = phero.getPmapXY(); 
        return {pmap_x, pmap_y, prev_pmap_x, prev_pmap_y};    //        Serial.println(pos.current_x)
    }
    
};

#endif