// Car.h - Header file for the Car class

#ifndef CAR_H  // Include guard to prevent multiple inclusions
#define CAR_H

#include <string>  // For std::string

class Car {
private:
    std::string color;  // Private member variable for color
    int speed;          // Private member variable for speed
    bool engineOn;      // Private member variable for engine state

public:
    // Constructor: Initializes the car
    Car(std::string carColor);

    // Destructor: Cleans up when the car is destroyed
    ~Car();

    // Public methods
    void startEngine();     // Starts the engine
    void stopEngine();      // Stops the engine
    void accelerate(int amount);  // Increases speed
    void brake(int amount);       // Decreases speed
    int getSpeed() const;         // Returns current speed
    std::string getColor() const; // Returns car color
};

#endif  // End of include guard