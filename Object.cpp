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
    { 1, 1, 1 },
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 },
    { 0, 1, 1 },
    { 1, 0, 1 },
    { 1, 1, 0 },
    { 1,.5f, 0 },
};

void draw_vector( const Vec3& p, const Vec3& vec, Color color )
{
    Vec3 v = vec * 0.1f;

    glColor3fv( colors[ color ] );
    glBegin( GL_LINE_LOOP );
        glVertex3f( p.x, p.y, p.z );
        glVertex3f( p.x + v.x, p.y + v.y, p.z + v.z );
    glEnd();
}




CollisionManifold project( const std::vector<Axis>& axes, const std::vector<Vec3>& vertices )
{
    Vec3 collision_normal;
    float collision_depth = 0x3f800000;

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
            return CollisionManifold();
        }

        float depth = std::min( std::abs( min_extent - axes[ axis ].max()), std::abs( max_extent - axes[ axis ].min()));
        if ( depth < collision_depth ) {
            collision_depth = depth;
            collision_normal = axes[ axis ].direction().normalise();
        }

        //std::cout << std::endl << std::endl;
    }

    return CollisionManifold( collision_normal, collision_depth );
}

#include <boost/foreach.hpp>
void Collision_Volume::draw() const
{
    glPushMatrix();

    glColor3fv( colors[ White ] );
    glBegin( GL_LINE_LOOP );
        BOOST_FOREACH( const Vec3& vertex, vertices )
            glVertex3f( vertex.x, vertex.y, -28.5f );
    glEnd();

    glPopMatrix();
}


void Object::draw() const
{
    GLfloat w2 = dimensions.x / 2.0f;
    GLfloat h2 = dimensions.y / 2.0f;

    glPushMatrix();

    glTranslatef( position().x, position().y, position().z );
    glRotatef( angle_degrees, 0,0,-1 );

    glColor3fv( colors[ color ] );
    ///glBegin( GL_QUADS );
    glBegin( GL_LINE_LOOP );
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

    Vec3 X( cos( deg_to_rad * angle_degrees ), -sin( deg_to_rad * angle_degrees ), 0 );
    Vec3 Y( sin( deg_to_rad * angle_degrees ), cos( deg_to_rad * angle_degrees ), 0 );

    X *= dimensions.x / 2;
    Y *= dimensions.y / 2;

    vertices.push_back( position() - X - Y );
    vertices.push_back( position() + X - Y );
    vertices.push_back( position() + X + Y );
    vertices.push_back( position() - X + Y );

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

    std::vector<Axis> axes;

    axes.push_back( Axis( Vec3( 1,0,0 ), vertices ));
    axes.push_back( Axis( Vec3( 0,1,0 ), vertices ));

    return Collision_Volume( axes, vertices );
}

float tyre_lateral_resistance( float angle_degrees, float weight_newtons )
{
    if ( angle_degrees <= 3.0f && angle_degrees >= -3.0f )
    {
        return angle_degrees * 0.4f * weight_newtons;
    }
    else
    {
        return ( -0.007f * angle_degrees + 1.221f ) * weight_newtons;
    }
}
