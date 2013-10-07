#ifndef _OBJECT_H_
#define _OBJECT_H_

#include "OpenGL.h"
#include "Vec3.h"


enum Color { White = 0, Red, Green, Blue, Purple, Yellow, Cyan, COLOR_COUNT };

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


float tyre_lateral_resistance( float angle_degrees, float weight_newtons );


enum Steerable { No_Steer = 0, Steer };

class Wheel {
public:
    Wheel( Steerable steer, Vec3 off )
    :   can_steer( steer ),
        offset( off )
    { ; }

    void apply_steering( float steer )
    {
        if ( can_steer )
            angle_relative_to_car = steer;
    }

    Vec3 forward_unit_vector( float car_angle )
    {
        float angle = car_angle + angle_relative_to_car;
        return Vec3( sin( deg_to_rad * angle ), cos( deg_to_rad * angle ), 0 );
    }

    Vec3 lateral_unit_vector( float car_angle )
    {
        float angle = car_angle + angle_relative_to_car;
        return Vec3( cos( deg_to_rad * angle ), -sin( deg_to_rad * angle ), 0 );
    }

    Vec3 calculate_velocity( Vec3 car_velocity, float car_angular_velocity )
    {
        Vec3 velocity = car_velocity;
        // using lateral
        velocity += lateral_unit_vector() * car_angular_velocity *
    }

private:
    Steerable can_steer;
    float angle_relative_to_car;
    Vec3 offset_from_car_centre;
};

class Car2D : public Object {
public:
    Car2D( Vec3 p, Vec3 d )
    :   Object( p, d, 10.0f, Purple ),
        mass( 500.0f ), // kg
        velocity( 0,0,0 ),
        forces( 0,0,0 )
    {
        angular_velocity = 0;
        torques = 0;
        steering_angle = 0;

        inertia = 0.04f * mass * dimensions.magnitude_squared();

        wheels.push_back( Wheel( No_Steer, d * Vec3(  0.5f, -0.5f, 0 )));
        wheels.push_back( Wheel( No_Steer, d * Vec3( -0.5f, -0.5f, 0 )));
        wheels.push_back( Wheel( Steer,    d * Vec3(  0.5f, 0.5f, 0 )));
        wheels.push_back( Wheel( Steer,    d * Vec3( -0.5f, 0.5f, 0 )));
    }

    Vec3 forward_unit_vector()
    {
        return Vec3( sin( deg_to_rad * angle ), cos( deg_to_rad * angle ), 0 );
    }

    Vec3 lateral_unit_vector()
    {
        return Vec3( cos( deg_to_rad * angle ), -sin( deg_to_rad * angle ), 0 );
    }

    void apply_drag()
    {
        float drag_coefficient = 0.4f;
        Vec3 drag = ( drag_coefficient * velocity.magnitude() * -velocity );

        forces += drag;
    }


    /// for each wheel
    ///    use car velocity & car angular velocity to
    ///        calculate wheel velocity
    ///    use wheel velocity & steering angle to
    ///        calculate wheel force
    ////    sum all wheel forces and apply to car forces & car torque

    ///  apply drag, drive & external forces to car forces & car torque
    ///  use car forces to calculate new car velocity
    //  use car torque to calculate new car angular velocity
    ///  store car velocity & car angular velocity for use next frame
    void update( float timestep )
    {
        draw_vector( pos, 0.1f * forces, Blue );

        unsigned int wheel_count = 4;

        /// for each wheel
        for ( unsigned int w = 0; w != wheel_count; ++w )
        {
            Vec3 wheel_world_position = pos + (wheels[ w ].offset.x - 0.1f) * lateral_unit_vector() + (wheels[ w ].offset.y - 0.1f) * forward_unit_vector();

            wheels[ w ].apply_steering( steering_angle );

            ///    use car velocity & car angular velocity to
            ///        calculate wheel velocity

            // wheel going same speed/direction as car
            Vec3 wheel_velocity = velocity;

           // wheels[ w ].calculate_velocity( velocity, lateral_unit_vector() * angular_velocity );

            // except when car is also rotating
            wheel_velocity += lateral_unit_vector() * angular_velocity * wheels[ w ].offset.y;

            float wheel_velocity_angle = wheel_steering_angle;

            if ( wheel_velocity.magnitude_squared() != 0.0f )
                acos( deg_to_rad * wheel_velocity.dot( Vec3( 0,1,0 )) / wheel_velocity.magnitude() );

   //         std::cout << "wheel_velocity_angle: " << wheel_velocity_angle << std::endl;

            ///    use wheel velocity & steering angle to
            ///        calculate wheel force

            Vec3 tyre_forward_unit_vector( sin( deg_to_rad * (wheel_steering_angle + angle) ), cos( deg_to_rad * (wheel_steering_angle + angle) ), 0 );
            Vec3 tyre_lateral_unit_vector( cos( deg_to_rad * (wheel_steering_angle + angle) ), -sin( deg_to_rad * (wheel_steering_angle + angle) ), 0 );

            float magnitude_of_velocity_in_tyre_direction = wheel_velocity.dot( tyre_forward_unit_vector );
            float magnitude_of_velocity_in_tyre_lateral   = wheel_velocity.dot( tyre_lateral_unit_vector );

            // lookup resistance on a sideways-moving tyre, from angle of tyre from direction of movement,
            // and assuming the wheels share the load of the car body equally
            Vec3 F_lat_wheel = magnitude_of_velocity_in_tyre_lateral * -tyre_lateral_unit_vector * tyre_lateral_resistance( wheel_velocity_angle - wheel_steering_angle + angle, 9.8f * mass / wheel_count );

   /*         std::cout << "lateral vector: " << magnitude_of_velocity_in_tyre_lateral * -tyre_lateral_unit_vector << std::endl;

            std::cout << "tyre_lateral_resistance: " << tyre_lateral_resistance( wheel_velocity_angle - ( wheel_steering_angle + angle ), 9.8f * mass / wheel_count ) << std::endl;
            std::cout << F_lat_wheel << std::endl << std::endl;
     */       // apply rolling resistance in direction
            float rr_coefficient = 12.8f;

            Vec3 F_long_wheel = rr_coefficient * -magnitude_of_velocity_in_tyre_direction * tyre_forward_unit_vector;

            ///    sum all wheel forces and apply to car forces & car torque
            // forces

         //   std::cout << F_lat_wheel << " + ";

      ///      draw_vector( wheel_world_position, tyre_forward_unit_vector, White );
            draw_vector( wheel_world_position, Vec3(1,0,0), Cyan );
            draw_vector( wheel_world_position, 0.001f * F_lat_wheel, Red );
            draw_vector( wheel_world_position, wheel_velocity, Yellow );


            forces += F_lat_wheel;
            forces += F_long_wheel;

            // distances from centre of mass
            Vec3 tyre_to_centre_of_mass = -wheels[ w ].offset;

            // find the point on the tyre forward vector that is closest to the centre of mass
            float forward_distance = tyre_to_centre_of_mass.dot( tyre_forward_unit_vector );
            float lateral_distance = tyre_to_centre_of_mass.dot( tyre_lateral_unit_vector );

  //          std::cout << "torque distances: " << forward_distance << ", " << lateral_distance << std::endl;

           /* Vec3 perpendicular_point = forward_distance * forward_unit_vector();

            // pythagoras
            Vec3 F_lat_distance = perpendicular_point - tyre_to_centre_of_mass;
            Vec3 F_long_distance = perpendicular_point;*/

            torques += F_lat_wheel.magnitude() * forward_distance;
            torques += F_long_wheel.magnitude() * lateral_distance;
        }

        draw_vector( pos + forward_unit_vector(), torques * 0.01f * lateral_unit_vector(), Purple );

   /*     std::cout << "Force: "  << forces  << std::endl;
        std::cout << "Torque: " << torques << std::endl;
*/

        ///  apply drag, drive & external forces to car forces & car torque
        apply_drag();

        ///  use car forces to calculate new car velocity
        Vec3 acceleration = forces / mass;
        velocity += acceleration * timestep;
        pos += velocity * timestep;

//        draw_vector( pos, acceleration, Blue );

    /*    std::cout << "Acceleration: ";
        threshold_print( std::cout, 0.001f, acceleration.magnitude(), "Constant Velocity" ) << std::endl;
        std::cout << "Velocity: ";
        threshold_print( std::cout, 0.001f, velocity.magnitude(),     "Stopped." ) << std::endl;
*/

        threshold_print( std::cout, 0.001f, torques ) << " / ";
        threshold_print( std::cout, 0.001f, inertia ) << " = ";
        threshold_print( std::cout, 0.001f, torques / inertia ) << std::endl;

        ///  use car torque to calculate new car angular velocity
        float angular_acceleration = torques / inertia;
        angular_velocity += angular_acceleration * timestep;
        angle += angular_velocity * timestep;

        std::cout << "Angular Velocity: ";
        threshold_print( std::cout, 0.001f, angular_velocity,     "Rotation Stopped." ) << std::endl;

        std::cout << std::endl;

        ///  store car velocity & car angular velocity for use next frame
        forces = Vec3(0,0,0);
        torques = 0;
    }

    void accelerate()
    {
        forces += forward_unit_vector() * 800.0f;
    }

    void brake()
    {
        forces += forward_unit_vector() * -200.0f;
    }

    void steer( float steering_change )
    {
        float steering_max = 60.0f;

        steering_angle += steering_change;

        if ( steering_angle < -steering_max )
            steering_angle = -steering_max;

        if ( steering_angle > steering_max )
            steering_angle = steering_max;
    }

    virtual void draw()
    {
        Object::draw();

        draw_vector( pos, velocity, Green );
      //  draw_vector( pos, forces, Red );
    }

private:
    std::vector<Wheel> wheels;

    float mass;
    Vec3 velocity;
    Vec3 forces;

    float inertia;
    float angular_velocity;
    float torques;

    float steering_angle;
};


#endif
