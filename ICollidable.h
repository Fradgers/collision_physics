#include <boost/shared_ptr.hpp>
#include "Vec3.h"

class Line_Segment;
class AA_Bounding_Box;
class Oriented_Bounding_Box;

class Collision_Volume {
public:
    virtual bool intersects( Collision_Volume& ) = 0;
    virtual bool intersects( Oriented_Bounding_Box& ) = 0;
    virtual bool intersects( AA_Bounding_Box& ) = 0;

    virtual ~Collision_Volume() = 0;
};

bool intersect_impl( const Oriented_Bounding_Box& O1, const Oriented_Bounding_Box& O2 );
bool intersect_impl( const Oriented_Bounding_Box& O, const AA_Bounding_Box& A );
bool intersect_impl( const AA_Bounding_Box& A1, const AA_Bounding_Box& A2 );
bool intersect_impl( const AA_Bounding_Box& A, const Line_Segment& L );
bool intersect_impl( const Line_Segment& L1, const Line_Segment& L2 );
bool intersect_impl( const Line_Segment& L, const Oriented_Bounding_Box& O );

/*class Line_Segment : public Collision_Volume {
public:
    Line_Segment( Vec3, Vec3 ) { }

    virtual bool intersects( Oriented_Bounding_Box* ) = 0;
    virtual bool intersects( AA_Bounding_Box* ) = 0;
};*/

#include <iostream>
class AA_Bounding_Box : public Collision_Volume {
public:
    AA_Bounding_Box( Vec3, Vec3 ) { }

    virtual bool intersects( Collision_Volume& other )
    {
        return other.intersects( *this );
    }

    virtual bool intersects( Oriented_Bounding_Box& other )  { std::cout << "A-O"; return intersect_impl( other, *this ); };
    virtual bool intersects( AA_Bounding_Box& other ) { std::cout << "A-A"; return intersect_impl( *this, other ); };
};

class Oriented_Bounding_Box : public Collision_Volume {
public:
    Oriented_Bounding_Box( Vec3 position, Vec3 dimensions, float angle )
    {
        Vec3 X( cos( angle ), sin( angle ), 0 );
        Vec3 Y( sin( angle ), cos( angle ), 0 );

        X *= dimensions.x / 2;
        Y *= dimensions.y / 2;

        corner[0] = position - X - Y;
        corner[1] = position + X - Y;
        corner[2] = position + X + Y;
        corner[3] = position - X + Y;

        compute_axes();
    }

    virtual bool intersects( Collision_Volume& other )
    {
        return other.intersects( *this );
    }

    virtual bool intersects( Oriented_Bounding_Box& other )  { std::cout << "O-O"; return intersect_impl( *this, other ); };
    virtual bool intersects( AA_Bounding_Box& other )  { std::cout << "O-A"; return intersect_impl( *this, other ); };

    Vec3 corner[4];
    Vec3 axis[2];
    float origin[2];

private:
    void compute_axes()
    {
        axis[0] = corner[1] - corner[0];
        axis[1] = corner[3] - corner[0];

        for ( int a = 0; a < 2; ++a )
        {
            axis[a] /= axis[a].magnitude_squared();
            origin[a] = corner[0].dot( axis[a] );
        }
    }
};


class ICollidable {
public:
    virtual boost::shared_ptr<Collision_Volume> collision_volume() = 0;
    virtual void draw() const = 0;

    virtual ~ICollidable() = 0;
};

