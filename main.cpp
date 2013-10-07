#include "Object.h"

const GLfloat depth = -30.0f;

#include <vector>
#include <boost/assign/std/vector.hpp>
#include <boost/foreach.hpp>
#include <iostream>

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
    OpenGL opengl;
    if ( ! opengl.good() ) return -1;

    using namespace boost::assign;

    Object obj( Vec3( 0, 0, depth ), Vec3( 2, 4, 0 ), 30.0f, Red );
    Car2D car( Vec3( -1.0f, -1, depth ), Vec3( 1, 3, 0 ) );

    std::vector<Vec3> positions;

    while ( ! glfwGetKey( GLFW_KEY_ESC ) )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();

        //std::cout << "Car initial position: " << car.position() << std::endl << std::endl;

        obj.draw();
        car.draw();
        car.update(0.05f);

        //std::cout << "\n************************************" << std::endl;

        if ( glfwGetKey( GLFW_KEY_UP )) { std::cout << "Accelerating." << std::endl; car.accelerate(); }
        if ( glfwGetKey( GLFW_KEY_DOWN )) { std::cout << "Braking." << std::endl; car.brake(); }
        if ( glfwGetKey( GLFW_KEY_RIGHT )) car.steer(5.0f);
        if ( glfwGetKey( GLFW_KEY_LEFT )) car.steer(-5.0f);

        if ( glfwGetKey( GLFW_KEY_SPACE ))
            car.handbrake( true );
        else
            car.handbrake( false );


        positions.push_back( car.position() );

        glPushMatrix();

        glColor3f( 1.0f, 1.0f, 1.0f );
        glBegin( GL_LINES );
            BOOST_FOREACH( const Vec3& position, positions )
                glVertex3f( position.x, position.y, depth );
        glEnd();

        glPopMatrix();

        CollisionManifold collision = obj.collision_volume().intersects( car.collision_volume());

        if ( collision.collision_detected )
        {
            std::cout << "COLLISION!!" << std::endl;
            draw_vector(car.position(), collision.normal * collision.depth, Red);
            car.resolve_collision( collision );
            obj.collision_volume().draw();
            car.collision_volume().draw();
        }

        glfwSwapBuffers();
    }
}
