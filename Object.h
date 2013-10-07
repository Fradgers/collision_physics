#ifndef _OBJECT_H_
#define _OBJECT_H_

#include "OpenGL.h"
#include "Vec3.h"

enum Color { White = 0, Red, Green, Blue, COLOR_COUNT };

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

private:
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


#endif
