#include "Object.h"

std::ostream& threshold_print( std::ostream& stream, float delta, float value, const std::string& message )
{
    if ( value < delta && value > -delta )
    {
        if ( value > 0 ) std::cout << '+';
        if ( value < 0 ) std::cout << '-';
        stream << message << std::flush;
    }
    else
    {
        std::fixed( std::cout );
        stream << std::setprecision(2) << value << std::flush;
    }

    return stream;
}

std::ostream& threshold_print( std::ostream& stream,  float delta, float value )
{
    return threshold_print( stream, delta, value, "NEG" );
}

GLfloat colors[COLOR_COUNT][3] = {
    { 1,1,1 },
    { 1,0,0 },
    { 0,1,0 },
    { 0,0,1 },
    { 1,0,1 },
};

bool project( const std::vector<Axis>& axes, const std::vector<Vec3>& vertices )
{
    // for both axes of this OBB
    for ( unsigned int axis = 0; axis != axes.size(); ++axis )
    {
        axes[ axis ].print();

        // project vertices onto the axis
        // find min and max positions on axis
        float extent = axes[ axis ].extent( vertices[0] );

        float min_extent = extent;
        float max_extent = extent;

        for ( unsigned int vertex = 1; vertex != vertices.size(); ++vertex )
        {
            extent = axes[ axis ].extent( vertices[ vertex ] );

            min_extent = std::min( min_extent, extent );
            max_extent = std::max( max_extent, extent );
        }

        // no overlap, bail out early
        if (
            ( min_extent > axes[ axis ].max() ) ||
            ( max_extent < axes[ axis ].min() )
        )
        {
            std::cout << "No Collision." << std::endl;
            return false;
        }
    }

    std::cout << "COLLISION!!" << std::endl;
    return true;
}

#include <boost/foreach.hpp>
void Collision_Volume::draw() const
{
    glPushMatrix();

    glColor3fv( colors[ White ] );
    glBegin( GL_LINE_LOOP );
        BOOST_FOREACH( const Vec3& vertex, vertices )
            glVertex3f( vertex.x, vertex.y, -19.9f );
    glEnd();

    glPopMatrix();
}


void Object::draw() const
{
    GLfloat w2 = dimensions.x / 2.0f;
    GLfloat h2 = dimensions.y / 2.0f;

    glPushMatrix();

    glTranslatef( position().x, position().y, position().z );
    glRotatef( angle, 0,0,1 );

    glColor3fv( colors[ color ] );
    glBegin( GL_QUADS );
        glVertex3f( -w2, h2, 0 );
        glVertex3f( -w2, -h2, 0 );
        glVertex3f( w2, -h2, 0 );
        glVertex3f( w2, h2, 0 );
    glEnd();

    glPopMatrix();
}

Collision_Volume Object::collision_volume()
{
    std::vector<Vec3> vertices;
    //angle += 0.003f;

    Vec3 X( cos( deg_to_rad * angle ), sin( deg_to_rad * angle ), 0 );
    Vec3 Y( -sin( deg_to_rad * angle ), cos( deg_to_rad * angle ), 0 );

    X *= dimensions.x / 2;
    Y *= dimensions.y / 2;

    vertices.push_back( position() - X - Y );
    vertices.push_back( position() + X - Y );
    vertices.push_back( position() + X + Y );
    vertices.push_back( position() - X + Y );

    /*glPushMatrix();

    glColor3fv( colors[ Blue ] );
    glBegin( GL_LINE_LOOP );
        BOOST_FOREACH( const Vec3& vertex, vertices )
            glVertex3f( vertex.x, vertex.y, -19.8f );
    glEnd();

    glPopMatrix();*/

    std::vector<Axis> axes;

    axes.push_back( Axis( vertices[1] - vertices[0], vertices ));
    axes.push_back( Axis( vertices[3] - vertices[0], vertices ));

    return Collision_Volume( axes, vertices );
}


void Block::draw() const
{
    GLfloat w2 = dimensions.x / 2.0f;
    GLfloat h2 = dimensions.y / 2.0f;

    glPushMatrix();

    glTranslatef( position().x, position().y, position().z );
    //glRotatef( -angle, 0,0,1 );

    glColor3fv( colors[ color ] );
    glBegin( GL_QUADS );
        glVertex3f( -w2, h2, 0 );
        glVertex3f( -w2, -h2, 0 );
        glVertex3f( w2, -h2, 0 );
        glVertex3f( w2, h2, 0 );
    glEnd();

    glPopMatrix();
}

Collision_Volume Block::collision_volume()
{
    std::vector<Vec3> vertices;

    float w2 = dimensions.x / 2.0f;
    float h2 = dimensions.y / 2.0f;

    vertices.push_back( position() + Vec3( -w2, -h2, 0 ));
    vertices.push_back( position() + Vec3( -w2, h2, 0 ));
    vertices.push_back( position() + Vec3( w2, h2, 0 ));
    vertices.push_back( position() + Vec3( w2, -h2, 0 ));

    /*glPushMatrix();

    glColor3fv( colors[ Green ] );
    glBegin( GL_LINE_LOOP );
        BOOST_FOREACH( const Vec3& vertex, vertices )
            glVertex3f( vertex.x, vertex.y, -19.8f );
    glEnd();

    glPopMatrix();*/

    std::vector<Axis> axes;

    axes.push_back( Axis( Vec3( 1,0,0 ), vertices ));
    axes.push_back( Axis( Vec3( 0,1,0 ), vertices ));

    return Collision_Volume( axes, vertices );
}

