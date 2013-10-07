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

    std::vector< ICollidable* > objects;
    objects +=
        new Block( Vec3( 4, 0, depth ), Vec3( 1, 5, 0 ), Blue ),
        new Object( Vec3( 0, 0, depth ), Vec3( 2, 2, 0 ), 30.0f, Red ),
        new Object( Vec3( 2, 0, depth ), Vec3( 3, 1, 0 ), 45.0f, Green ),
        new Block( Vec3( 1, -2.5f, depth ), Vec3( 2, 2, 0 ), Red )
    ;

    while ( ! glfwGetKey( GLFW_KEY_ESC ) )
    {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();

        for ( unsigned int a = 0; a != objects.size(); ++a )
            for ( unsigned int b = a+1; b != objects.size(); ++b )
            {
                if ( objects[ a ]->collision_volume().intersects(
                    objects[ b ]->collision_volume()
                ))
                {
                    objects[ a ]->collision_volume().draw();
                    objects[ b ]->collision_volume().draw();
//                    std::cout << " " << a << " " << b;
                }
 //                std::cout << std::endl;
            }

        BOOST_FOREACH( const ICollidable* obj, objects )
            obj->draw();

        if ( glfwGetKey( GLFW_KEY_UP )) objects[0]->move_by( Vec3( 0, 0.01f, 0 ));
        if ( glfwGetKey( GLFW_KEY_DOWN )) objects[0]->move_by( Vec3( 0, -0.01f,0 ));
        if ( glfwGetKey( GLFW_KEY_RIGHT )) objects[0]->move_by( Vec3( 0.01f, 0, 0 ));
        if ( glfwGetKey( GLFW_KEY_LEFT )) objects[0]->move_by( Vec3( -0.01f, 0, 0 ));

        glfwSwapBuffers();
    }

    BOOST_FOREACH( const ICollidable* obj, objects )
        delete obj;

}
