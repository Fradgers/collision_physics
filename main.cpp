#include "Object.h"

GLfloat depth = 40.0f;

#include <vector>
#include <boost/assign/std/vector.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <assert.h>

/**
    power_weight_ratio
    maximum_thrust
    maximum_brake

    steering_rate
    steering_lock

    mass

    rolling_friction_coefficient
    lateral_friction_coefficient
    drag_coefficient
*/


int main( int argc, char** argv )
{
    // setup window
    OpenGL opengl;
    if ( ! opengl.good() ) return -1;

    // initialise objects
    Object obj( Vec3( 3, 0, 0.0f ), Vec3( 4, 5, 0 ), 45.0f, Red );
    Car2D car( Vec3( 0.0f, -0.1f, 0.0f ), Vec3( 1, 2, 0 ) );

    Vec3 camera( 0.0f, 0.0f, depth );
    GLfloat camera_orientation = 0.0f;

    std::vector<Vec3> positions;

    // setup collision handling functions
    CollisionResolver collision_resolver;
    collision_resolver.subscribe( "Car2d", "Object", resolve_car2d_object );
    collision_resolver.subscribe( "Car2d", "Car2d", resolve_car2d_car2d );

    while ( ! glfwGetKey( GLFW_KEY_ESC ) )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();

		// have camera follow car and increase field of view at faster speeds
        camera_orientation = car.orientation();
        glRotatef( camera_orientation, 0,0,1.0f );

		depth = std::max( 40.0f, car.get_velocity().magnitude() * 2.0f );
        camera = car.position() + Vec3( 0.0f, 0.0f, depth ) + car.forward_unit_vector() * 2.0f;
        glTranslatef( -camera.x, -camera.y, -camera.z );

		// update and draw game objects
        obj.draw();
        car.draw();
        car.update(0.05f);

        car.centre_steering();

        // handle input
        if ( glfwGetKey( GLFW_KEY_UP )) { car.accelerate(); }
        if ( glfwGetKey( GLFW_KEY_DOWN )) { car.brake(); }
        if ( glfwGetKey( GLFW_KEY_RIGHT )) car.steer(3.0f);
        if ( glfwGetKey( GLFW_KEY_LEFT )) car.steer(-3.0f);

        if ( glfwGetKey( GLFW_KEY_SPACE ))
            car.handbrake( true );
        else
            car.handbrake( false );

        // draw white dotted line showing path travelled by car
        positions.push_back( car.position() );

        glPushMatrix();
        glColor3f( 1.0f, 1.0f, 1.0f );
        glBegin( GL_LINES );
            BOOST_FOREACH( const Vec3& position, positions )
                glVertex3f( position.x, position.y, 0.0f );
        glEnd();
        glPopMatrix();

        // check for collisions between car and object
        CollisionManifold collision = obj.collision_volume().intersects( car.collision_volume());

        if ( collision.collision_detected )
        {
            collision_resolver.resolve( car, obj, collision );

            obj.collision_volume().draw(); // highlight collision in white
            car.collision_volume().draw();
        }

        glfwSwapBuffers();
    }
}
