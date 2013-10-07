#ifndef _OBJECT_H_
#define _OBJECT_H_

#include "OpenGL.h"
#include "Vec3.h"

enum Color { White = 0, Red, Green, Blue, Purple, COLOR_COUNT };

#include <vector>
#include <limits>
#include <iostream>
class Axis {
public:
    Axis( const Vec3& d, const std::vector<Vec3>& points )
    :   direction(d),
        min_extent( std::numeric_limits<float>::max() ),
        max_extent( -min_extent )
    {
        for ( unsigned int p = 0; p != points.size(); ++p )
        {
            float e = extent( points[ p ] );

            if ( min_extent > e ) min_extent = e;
            if ( max_extent < e ) max_extent = e;
        }
    }

    float max() const { return max_extent; }
    float min() const { return min_extent; }

    float extent( const Vec3& point ) const
    {
        return direction.dot( point );
    }

    void print() const
    {
//        std::cout << direction << " Min: " << min() << " Max: " << max() << std::endl;
    }

private:
    Vec3 direction;
    float min_extent;
    float max_extent;
};


bool project( const std::vector<Axis>& axes, const std::vector<Vec3>& vertices );


class Collision_Volume {
public:
    Collision_Volume( const std::vector<Axis>& as, const std::vector<Vec3>& vs )
    :   axes(as), vertices(vs)
    { ; }

    bool intersects( const Collision_Volume& other )
    {
        return ( project( axes, other.vertices ) && project( other.axes, vertices ));
    }

    void draw() const;

private:
    std::vector<Axis> axes;
    std::vector<Vec3> vertices;
};


class ICollidable {
public:
    virtual Collision_Volume collision_volume();
    virtual void draw() const;
    virtual Vec3 position() const;
    virtual void move_by( const Vec3& vec );
};


class Object : public ICollidable {
public:
    Object( Vec3 p, Vec3 d, float a, Color c )
    :   pos(p), dimensions(d), angle(a), color(c)
    { ; }

    virtual Collision_Volume collision_volume();
    void draw() const;
    Vec3 position() const { return pos; }
    void move_by( const Vec3& vec ) { pos += vec; }

protected:
    Vec3 pos;
    Vec3 dimensions;
    float angle;
    Color color;
};


class Block : public ICollidable {
public:
    Block( Vec3 p, Vec3 d, Color c )
    :   pos(p), dimensions(d), color(c)
    { ; }

    virtual Collision_Volume collision_volume();
    void draw() const;
    Vec3 position() const { return pos; }
    void move_by( const Vec3& vec ) { pos += vec; }

private:
    Vec3 pos;
    Vec3 dimensions;
    Color color;
};


#include <iomanip>
std::ostream& threshold_print( std::ostream& stream, float delta, float value, const std::string& message );
std::ostream& threshold_print( std::ostream& stream, float delta, float value );


enum Power { UNPOWERED = 0, POWERED };

class Wheel {
public:
    Wheel( Vec3 offset, float angle )
    :   torque(0),
        angular_velocity(0),
        wheel_radius(0.5f),
        wheel_inertia(
            0.05f/12.0f * ( 3 * wheel_radius * wheel_radius) )
    { ; }

    void apply_torque( float torque_component )
    {
        torque += torque_component;
    }

    float resultant_force_on_chassis( float ground_speed, float mass, float timestep )
    {
        //std::cout << "Ratio: " << mass << " : " << wheel_inertia / wheel_radius << std::endl;

        // wheel is pointing towards positive Y
        //Vec3 forward_vector( 0,1,0 );
        angular_velocity *= 0.9f;

        float wheel_speed = /*forward_vector*/1 * angular_velocity * wheel_radius;
        float relative_velocity = ground_speed - wheel_speed;

        // if ground is moving faster than wheels are turning, relative_velocity is positive
        // if ground is moving slower than wheels are turning, relative_velocity is negative

        ///float friction = -relative_velocity;
        float static_friction_coefficient = 1.0f;
        float max_friction = static_friction_coefficient * mass * 9.8f;
        float friction = 0.0f;

        /// TODO: use torque to calculate opposing friction?

        friction = relative_velocity;
       /* if ( relative_velocity >= 0 )
        {
            friction = 100 * relative_velocity;//std::min( relative_velocity, max_friction );
        }
        else
        {
            friction = -1000;//std::max( relative_velocity, -max_friction );
        }*/

        // if ground is moving faster than wheels are turning, friction accelerates the wheels
        // if ground is moving slower than wheels are turning, friction decelerates the wheels

        if ( torque < friction * wheel_radius )
            torque -= friction * wheel_radius;

        // F=MA
        float angular_acceleration = torque / wheel_inertia;

        angular_velocity += angular_acceleration * timestep;

        std::cout << std::setprecision(4);
        std::cout << "Torque: ";
        threshold_print( std::cout, 0.01f, torque );
        std::cout << std::endl;
        std::cout << "Speed: " << "Wheel(";
        threshold_print( std::cout, 0.01f, wheel_speed );
        std::cout << ", ";
        threshold_print( std::cout, 0.01f, angular_velocity * wheel_radius );
        std::cout << "), Ground(";
        threshold_print( std::cout, 0.01f, ground_speed );
        std::cout << ")" << std::endl;

        std::cout << "Result: RelVel(";
        threshold_print( std::cout, 0.1f, relative_velocity );
        std::cout << ") Friction(";
        threshold_print( std::cout, 0.1f, -friction );
        std::cout << ")" << std::endl;

        if ( friction == max_friction )
            std::cout << "+max_friction (wheelspin)" << std::endl;
        if ( friction == -max_friction )
            std::cout << "-max_friction (wheelspin)" << std::endl;

        torque = 0;

        // friction is the force which accelerates or decelerates the car as a whole
        return -friction;
    }

private:
    float torque;
    float angular_velocity;
    float wheel_radius;
    float wheel_inertia;
};

class UniCar : public Object {
public:
    UniCar( Vec3 p, Vec3 d )
    :   Object( p, d, 0.0f, Purple ),
        wheel( d/2, 0.0f ),
        mass( 5.0f ), // kg
        velocity( 0 )
    { ; }

    void update( float timestep )
    {
        float resultant_force = wheel.resultant_force_on_chassis( velocity, mass, timestep );
        float acceleration = resultant_force / mass;

        velocity += acceleration * timestep;

        std::cout << "Acceleration: ";
        threshold_print( std::cout, 0.001f, acceleration, "Constant Velocity" ) << std::endl;

        std::cout << "Velocity: ";
        threshold_print( std::cout, 0.001f, velocity,     "Stopped." ) << std::endl;

        pos += Vec3( 0, velocity * timestep, 0 );

        std::cout << std::endl;
    }

    void accelerate()
    {
        wheel.apply_torque(20.0f);
    }

    void brake()
    {
        wheel.apply_torque(-5.0f);
    }

private:
    Wheel wheel;
    float mass;

    float velocity;
};


#endif
