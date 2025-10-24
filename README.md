# PheroLite

**Lightweight Virtual Pheromone Class for Bio-inspired Swarm Coordination**

A C++ library that implements virtual pheromone-based navigation for swarms. This work is based on a previous paper [1] and simplified to operate with messages as small as 12 bytes. Robots deposit virtual pheromones in a discretised grid map and use local pheromone concentrations to make decentralised movement decisions. Each agent has its own local pheromone map which is updated by itself and asynchronously with neighbouring robots if within range. 

An application of this library for task allocation with a swarm of ground-based robots is detailed in [MSc_Thesis.pdf](https://github.com/user-attachments/files/23123787/MSc_Thesis.pdf).


## Features

- **Virtual pheromone map**: Made from a 2D array of floats where the integer value represents pheromone type and the decimal represents the pheromone concentration. The map size and cell size are configurable (default 50×50 grid with 20mm cells = 1m × 1m area).
- **Two pheromone types**: Repulsive (avoidance) and attractive (following), can easily be changed for any desired purpose.
- **ESP-NOW compatible messaging** with 5 different message types for communication
- **Boundary avoidance** to keep robots within the virtual map area
- **Configurable parameters**: deposition radius, evaporation rates, message types
- **Memory efficient**: ~10KB RAM usage for 50×50 map

## Hardware 
- Tested on an ESP32-S3 microcontroller (M5 Stamp Fly drone).
- Used ESP-NOW but other wireless communication would work. With ESP-NOW, messages were broadcasted without a MAC address to maintain a decentralised swarm system.

## Quick Start

### 1. Include the Library
```cpp
#include "phero.hpp"
```
### 2. Create and Initialise
```cpp
// Constructor: cell_width(mm), start_x, start_y, evaporation_rate, radius, pheromone_type
Phero_c phero(20, 25, 25, 0.036, 1, 1);

void setup() {
    phero.initialise();  // Reset map and position
}
```
### 3. Main Loop
 ```cpp
void loop() {
    // Get robot position from sensors
    float x = getPositionX();    // mm from origin
    float y = getPositionY();    // mm from origin  
    float theta = getYaw();      // radians
    
    // Update pheromone system
    phero.update(x, y, theta);
    
    // Get heading at desired frequency
    float target_heading = phero.ThetaDemand();  // radians
    
    // Send to flight controller
    setDesiredHeading(target_heading);
    
    // Prepare message for swarm communication
    phero.msgUpdate();
    // Send phero.p_msg via ESP-NOW
}
 ```
### 4. Process Incoming Messages
```cpp
void onESPNowReceive(const uint8_t* data ... ) {
    phero.msgDecode((char*)data);  // Process received pheromone data
}
```
### Configuration
Constructor Parameters
```cpp
Phero_c phero(
    cell_width,     // Cell size in mm (default: 20mm)
    start_x,        // Starting X grid position (default: centre)
    start_y,        // Starting Y grid position (default: centre)
    evaporation,    // Evaporation rate 0.0-1.0 (default: 0.036)
    radius,         // Pheromone deposition radius in cells (default: 1)
    phero_type      // 0=no trail, 1=repulsive, 2=attractive (default: 1)
);
```
### Runtime Configuration
```cpp
phero.changePhero(2);           // Switch to attractive pheromones
phero.changeMsgType(1);         // Change message type (0-4)
phero.changeN_Rand_Cells(7);    // Set random cells in messages
```
| Msg Type | Description | Size | Content |
|------|-------------|------|---------|
| 0 | Local 3×3 grid | 12 bytes | ID + position + 9 cells |
| 1 | Local 5×5 grid | 28 bytes | ID + position + 25 cells |
| 2 | Random cells | Variable | ID + N random high-value cells |
| 3 | Hybrid sequential | Alternating | Sends type 0, then type 2 |
| 4 | Hybrid combination | 25 bytes | Type 0 + random cells combined |

### API Reference
Core Functions
```cpp
update(x, y, theta) - Update robot position and process pheromones
ThetaDemand() - Get desired heading in radians
msgUpdate() - Prepare message for transmission
msgDecode(data) - Process received pheromone message
```
### Getters
```cpp
getPmapXY() - Current grid position
getPmapCell(x, y) - Get pheromone value at coordinates
getMapSize() - Get map dimensions
```
### Visualisation
```cpp
// Print pheromone map in x-y graph format
for (int y = phero.getMapSize() - 1; y >= 0; y--) {
    for (int x = 0; x < phero.getMapSize(); x++) {
        float value = phero.getPmapCell(x, y);
        Serial.printf("%4.1f ", value);
    }
    Serial.println();
}
```
### Pheromone Encoding
The system uses an encoding scheme where each cell stores:

Integer part: Pheromone type and source (local vs neighbour)
Decimal part: Concentration (0.0-1.0)
| Range | Type |
|-------|------|
| 0.0-1.0 | Local repulsive |
| 2.0-3.0 | Local attractive |
| 10.0-11.0 | Neighbour repulsive |
| 12.0-13.0 | Neighbour attractive |

### Future Work
This system could be improved / expanded in numerous ways, these are a few that I would like to implement!

1) Find more optimal message types for communicating pheromone information to neighbours
2) Test different types of pheromones
3) Find a memory efficient method for creating a 3D pheromone map.
4) Find a method for context-dependent evaporation rate

### References
[1] C. R. Tinoco and G. M. B. Oliveira, ‘PheroCom: Decentralised and asynchronous swarm robotics coordination based on virtual pheromone and vibroacoustic communication’, Feb. 27, 2022, arXiv: arXiv:2202.13456. Accessed: Nov. 13, 2023. [Online]. Available: http://arxiv.org/abs/2202.13456
