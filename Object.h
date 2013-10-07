#ifndef _OBJECT_H_
#define _OBJECT_H_

#include "OpenGL.h"
#include "Vec3.h"


enum Color { White = 0, Red, Green, Blue, Purple, COLOR_COUNT };

void draw_vector( const Vec3& p, const Vec3& v, Color color = White );



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
    virtual void draw() const;
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


class UniCar : public Object {
public:
    UniCar( Vec3 p, Vec3 d )
    :   Object( p, d, 0.0f, Purple ),
        mass( 5.0f ), // kg
        velocity( 0 ),
        forces( 0 )
    { ; }

    void update( float timestep )
    {
        float friction = -velocity;
        forces += friction;

        float acceleration = forces / mass;

        velocity += acceleration * timestep;

        std::cout << "Acceleration: ";
        threshold_print( std::cout, 0.001f, acceleration, "Constant Velocity" ) << std::endl;

        std::cout << "Velocity: ";
        threshold_print( std::cout, 0.001f, velocity,     "Stopped." ) << std::endl;

        pos += Vec3( 0, velocity * timestep, 0 );

        std::cout << std::endl;

        forces = 0;
    }

    void accelerate()
    {
        forces += 5.0f;
    }

    void brake()
    {
        forces -= 2.0f;
    }

private:
    float mass;
    float velocity;
    float forces;
};


class Car2D : public Object {
public:
    Car2D( Vec3 p, Vec3 d )
    :   Object( p, d, 10.0f, Purple ),
        mass( 5.0f ), // kg
        velocity( 0,0,0 ),
        forces( 0,0,0 )
    {
        inertia = 0.8f * mass * dimensions.magnitude_squared();
    }

    Vec3 forward_unit_vector()
    {
        return Vec3( sin( deg_to_rad * angle ), cos( deg_to_rad * angle ), 0 );
    }

    void apply_drag()
    {
        float drag_coefficient = 0.4f;
        float rr_coefficient = 12.8f;
        Vec3 drag = ( drag_coefficient * velocity.magnitude() * -velocity );
        Vec3 rr = ( rr_coefficient * -velocity );

        forces += drag + rr;
    }

    void update( float timestep )
    {
        apply_drag();

        // calculate Vcar
        Vec3 acceleration = forces / mass;
        velocity += acceleration * timestep;
        pos += velocity * timestep;

        draw_vector( pos, acceleration, Blue );

        std::cout << "Acceleration: ";
        threshold_print( std::cout, 0.001f, acceleration.magnitude(), "Constant Velocity" ) << std::endl;
        std::cout << "Velocity: ";
        threshold_print( std::cout, 0.001f, velocity.magnitude(),     "Stopped." ) << std::endl;

        // calculate AVcare
        angular_velocity *= 0.9f;
        angle += angular_velocity * timestep;

        std::cout << "Angular Velocity: ";
        threshold_print( std::cout, 0.001f, angular_velocity,     "Rotation Stopped." ) << std::endl;

        std::cout << std::endl;

        forces = Vec3(0,0,0);
        torques = 0;
    }

    void accelerate()
    {
        forces += forward_unit_vector() * 80.0f;
    }

    void brake()
    {
        forces += forward_unit_vector() * -20.0f;
    }

    void steer( float angular_acceleration )
    {
        float cos_angle_between_facing_and_velocity = ( forward_unit_vector().dot( velocity ));
        angular_velocity += angular_acceleration * std::max( 0.1f, cos_angle_between_facing_and_velocity );
        //angular_velocity += angular_acceleration * velocity.magnitude();
    }

    virtual void draw()
    {
        Object::draw();

        draw_vector( pos, velocity, Green );
      //  draw_vector( pos, forces, Red );
    }

private:
    float mass;
    Vec3 velocity;
    Vec3 forces;

    float inertia;
    float angular_velocity;
    float torques;
};


#endif
