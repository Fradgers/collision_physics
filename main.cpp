#include "Object.h"

const GLfloat depth = -20.0f;

#include <vector>
#include <boost/assign/std/vector.hpp>
#include <boost/foreach.hpp>
#include <iostream>

int main( int argc, char** argv )
{
    OpenGL opengl;
    if ( ! opengl.good() ) return -1;

    using namespace boost::assign;

    Object obj( Vec3( 0, 0, depth ), Vec3( 2, 2, 0 ), 30.0f, Red );
    Car2D car( Vec3( 4, 0, depth ), Vec3( 1, 3, 0 ) );

    while ( ! glfwGetKey( GLFW_KEY_ESC ) )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();

        car.update(0.01f);

        std::cout << "\n************************************" << std::endl;

        if ( glfwGetKey( GLFW_KEY_UP )) { std::cout << "Accelerating." << std::endl; car.accelerate(); }
        if ( glfwGetKey( GLFW_KEY_DOWN )) { std::cout << "Braking." << std::endl; car.brake(); }
        if ( glfwGetKey( GLFW_KEY_RIGHT )) car.steer(2.0f);
        if ( glfwGetKey( GLFW_KEY_LEFT )) car.steer(-2.0f);

        obj.draw();
        car.draw();

        if ( obj.collision_volume().intersects(
            car.collision_volume()
        ))
        {
            obj.collision_volume().draw();
            car.collision_volume().draw();
//                    std::cout << " " << a << " " << b;
        }

        glfwSwapBuffers();
    }

    ///while ( ! glfwGetKey( GLFW_KEY_SPACE ) )
    { ; }
}
