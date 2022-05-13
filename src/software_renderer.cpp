#include "software_renderer.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>

#include "triangulation.h"

using namespace std;

namespace CMU462 {


// Implements SoftwareRenderer //

void SoftwareRendererImp::draw_svg( SVG& svg ) {

  // set top level transformation
  transformation = svg_2_screen;

  // draw all elements
  for ( size_t i = 0; i < svg.elements.size(); ++i ) {
    draw_element(svg.elements[i]);
  }

  // draw canvas outline
  Vector2D a = transform(Vector2D(    0    ,     0    )); a.x--; a.y--;
  Vector2D b = transform(Vector2D(svg.width,     0    )); b.x++; b.y--;
  Vector2D c = transform(Vector2D(    0    ,svg.height)); c.x--; c.y++;
  Vector2D d = transform(Vector2D(svg.width,svg.height)); d.x++; d.y++;

  rasterize_line(a.x, a.y, b.x, b.y, Color::Black);
  rasterize_line(a.x, a.y, c.x, c.y, Color::Black);
  rasterize_line(d.x, d.y, b.x, b.y, Color::Black);
  rasterize_line(d.x, d.y, c.x, c.y, Color::Black);

  // resolve and send to render target
  resolve();

}

void SoftwareRendererImp::set_sample_rate( size_t sample_rate ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->sample_rate = sample_rate;
  sampleHeight = target_h * sample_rate;
  sampleWidth = target_w * sample_rate;
  sample_buffer = vector<Color>(sampleHeight * sampleWidth, Color(1.0, 1.0, 1.0, 1.0));
}

void SoftwareRendererImp::set_render_target( unsigned char* render_target,
                                             size_t width, size_t height ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->render_target = render_target;
  this->target_w = width;
  this->target_h = height;
  // Set sampler buffer
  sampleHeight = target_h * sample_rate;
  sampleWidth = target_w * sample_rate;
  sample_buffer = std::vector<Color>(sampleHeight * sampleWidth, Color(1.0, 1.0, 1.0, 1.0));
}

void SoftwareRendererImp::draw_element( SVGElement* element ) {

  // Task 5 (part 1):
  // Modify this to implement the transformation stack

  switch(element->type) {
    case POINT:
      draw_point(static_cast<Point&>(*element));
      break;
    case LINE:
      draw_line(static_cast<Line&>(*element));
      break;
    case POLYLINE:
      draw_polyline(static_cast<Polyline&>(*element));
      break;
    case RECT:
      draw_rect(static_cast<Rect&>(*element));
      break;
    case POLYGON:
      draw_polygon(static_cast<Polygon&>(*element));
      break;
    case ELLIPSE:
      draw_ellipse(static_cast<Ellipse&>(*element));
      break;
    case IMAGE:
      draw_image(static_cast<Image&>(*element));
      break;
    case GROUP:
      draw_group(static_cast<Group&>(*element));
      break;
    default:
      break;
  }

}


// Primitive Drawing //

void SoftwareRendererImp::draw_point( Point& point ) {

  Vector2D p = transform(point.position);
  rasterize_point( p.x, p.y, point.style.fillColor );

}

void SoftwareRendererImp::draw_line( Line& line ) { 

  Vector2D p0 = transform(line.from);
  Vector2D p1 = transform(line.to);
  rasterize_line( p0.x, p0.y, p1.x, p1.y, line.style.strokeColor );

}

void SoftwareRendererImp::draw_polyline( Polyline& polyline ) {

  Color c = polyline.style.strokeColor;

  if( c.a != 0 ) {
    int nPoints = polyline.points.size();
    for( int i = 0; i < nPoints - 1; i++ ) {
      Vector2D p0 = transform(polyline.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polyline.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_rect( Rect& rect ) {

  Color c;
  
  // draw as two triangles
  float x = rect.position.x;
  float y = rect.position.y;
  float w = rect.dimension.x;
  float h = rect.dimension.y;

  Vector2D p0 = transform(Vector2D(   x   ,   y   ));
  Vector2D p1 = transform(Vector2D( x + w ,   y   ));
  Vector2D p2 = transform(Vector2D(   x   , y + h ));
  Vector2D p3 = transform(Vector2D( x + w , y + h ));
  
  // draw fill
  c = rect.style.fillColor;
  if (c.a != 0 ) {
    rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    rasterize_triangle( p2.x, p2.y, p1.x, p1.y, p3.x, p3.y, c );
  }

  // draw outline
  c = rect.style.strokeColor;
  if( c.a != 0 ) {
    rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    rasterize_line( p1.x, p1.y, p3.x, p3.y, c );
    rasterize_line( p3.x, p3.y, p2.x, p2.y, c );
    rasterize_line( p2.x, p2.y, p0.x, p0.y, c );
  }

}

void SoftwareRendererImp::draw_polygon( Polygon& polygon ) {

  Color c;

  // draw fill
  c = polygon.style.fillColor;
  if( c.a != 0 ) {

    // triangulate
    vector<Vector2D> triangles;
    triangulate( polygon, triangles );

    // draw as triangles
    for (size_t i = 0; i < triangles.size(); i += 3) {
      Vector2D p0 = transform(triangles[i + 0]);
      Vector2D p1 = transform(triangles[i + 1]);
      Vector2D p2 = transform(triangles[i + 2]);
      rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    }
  }

  // draw outline
  c = polygon.style.strokeColor;
  if( c.a != 0 ) {
    int nPoints = polygon.points.size();
    for( int i = 0; i < nPoints; i++ ) {
      Vector2D p0 = transform(polygon.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polygon.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_ellipse( Ellipse& ellipse ) {

  // Extra credit 

}

void SoftwareRendererImp::draw_image( Image& image ) {

  Vector2D p0 = transform(image.position);
  Vector2D p1 = transform(image.position + image.dimension);

  rasterize_image( p0.x, p0.y, p1.x, p1.y, image.tex );
}

void SoftwareRendererImp::draw_group( Group& group ) {

  for ( size_t i = 0; i < group.elements.size(); ++i ) {
    draw_element(group.elements[i]);
  }

}

// Rasterization //

// Samples a single point in the sample buffer
void SoftwareRendererImp::sample_point(float x, float y, Color color) {
    // Find screenspace coordinates
    float sx = x / target_w;
    float sy = y / target_h;

    // check bounds
    if (sx < 0 || sx >= 1.0) return;
    if (sy < 0 || sy >= 1.0) return;

    // Find sample buffer device coordinates, fill sample pixel
    int sampX = sx * (float)sampleWidth;
    int sampY = sy * (float)sampleHeight;
    sample_buffer[sampX + (sampY * sampleWidth)] = color;
}

// The input arguments in the rasterization functions 
// below are all defined in screen space coordinates

void SoftwareRendererImp::rasterize_point( float x, float y, Color color ) {

  // Find screenspace coordinates
  int px = floor(x);
  int py = floor(y);

  // check bounds
  if (px < 0 || px >= target_w) return;
  if (py < 0 || py >= target_h) return;

  int sampleX = sample_rate * px;
  int sampleY = sample_rate * py;

  // Fill sample_rate * sample_rate block associated with pixel in sample buffer
  for (int y = sampleY; y < sampleY + sample_rate; y++) {
      for (int x = sampleX; x < sampleX + sample_rate; x++) {
          sample_buffer[x + (y * sampleWidth)] = color;
      }
  }
  
}

// Rasterize a line using Bresenham's algorithm
void SoftwareRendererImp::rasterize_line( float x0, float y0,
                                          float x1, float y1,
                                          Color color) {
    // Task 2: 
    // Determine slope, choose which rasterization version to use
    float slope = (float)(y1 - y0) / (float)(x1 - x0);
    if (abs(slope) < 1.0)
        rasterize_line_low_slope(x0, y0, x1, y1, color);
    else
        rasterize_line_high_slope(x0, y0, x1, y1, color);
}

// Rasterizes when absolute line slope <= 1.0
void SoftwareRendererImp::rasterize_line_low_slope(float x0, float y0, float x1, float y1, Color color) {
    if (x0 > x1) {
        int temp = x1;
        x1 = x0;
        x0 = temp;
        temp = y1;
        y1 = y0;
        y0 = temp;
    }
    int deltaX = x1 - x0;
    int deltaY = y1 - y0;
    float iterStep = 1.0;
    if (deltaY < 0) {
        iterStep = -1.0;
        deltaY *= -1;
    }
    int epsilon = 2 * deltaY - deltaX;
    float y = y0;
    for (float x = std::floor(x0) + 0.5; x <= x1; x++) {
        rasterize_point(x, y, color);
        if (epsilon > 0) {
            y += iterStep;
            epsilon -= 2 * deltaX;
        }
        epsilon += 2 * deltaY;
    }
}

// Rasterizes when absolute line slope > 1.0
void SoftwareRendererImp::rasterize_line_high_slope(float x0, float y0, float x1, float y1, Color color) {
    if (y0 > y1) {
        int temp = y1;
        y1 = y0;
        y0 = temp;
        temp = x1;
        x1 = x0;
        x0 = temp;
    }
    int deltaX = x1 - x0;
    int deltaY = y1 - y0;
    // Use x as the dependant variable
    float iterStep = 1.0;
    if (deltaX < 0) {
        iterStep = -1.0;
        deltaX *= -1;
    }
    int epsilon = 2 * deltaX - deltaY;
    float x = x0;
    for (float y = std::floor(y0) + 0.5; y <= y1; y ++) {
        rasterize_point(x, y, color);
        if (epsilon > 0) {
            x += iterStep;
            epsilon -= 2 * deltaY;
        }
        epsilon += 2 * deltaX;
    }
}

// Pineda's edge function
// (A Parallel Algorithm for Polygon Rasterization, 1988)
// pX and pY are values of the point to test
// (x0, y0) and (x1, y1) are the vertices of the triangle edge to use in the test
bool edgeFunction(float x0, float y0, float x1, float y1, float xP, float yP) {
    // Triangle vertices are defined in clockwise order, which requires testing for results <= 0
    return ((xP - x0) * (y1 - y0) - (yP - y0) * (x1 - x0)) <= 0.0;
}

void SoftwareRendererImp::rasterize_triangle( float x0, float y0,
                                              float x1, float y1,
                                              float x2, float y2,
                                              Color color ) {
    // Task 3: 
    // Implement triangle rasterization
    // Create bounding box
    // Move by 0.5 to sample mid pixels
    float xMin = std::floor(std::min({ x0, x1, x2 })) + (0.5 / sample_rate);
    float yMin = std::floor(std::min({ y0, y1, y2 })) + (0.5 / sample_rate);
    float xMax = std::ceil(std::max({ x0, x1, x2 })) + (0.5 / sample_rate);
    float yMax = std::ceil(std::max({ y0, y1, y2 })) + (0.5 / sample_rate);

    // For testing points within the triangle's bounding box
    for (float y = yMin; y <= yMax; y += (1.0 / sample_rate)) {
        for (float x = xMin; x <= xMax; x += (1.0 / sample_rate)) {
            bool pointInside = true;
            // Test point against all three edges
            pointInside &= edgeFunction(x0, y0, x1, y1, x, y);
            pointInside &= edgeFunction(x1, y1, x2, y2, x, y);
            pointInside &= edgeFunction(x2, y2, x0, y0, x, y);

            if (pointInside)
                sample_point(x, y, color);
        }
    }
}



void SoftwareRendererImp::rasterize_image( float x0, float y0,
                                           float x1, float y1,
                                           Texture& tex ) {
  // Task 6: 
  // Implement image rasterization

}

// resolve samples to render target
void SoftwareRendererImp::resolve( void ) {

  // Task 4: 
  // Implement supersampling
  // You may also need to modify other functions marked with "Task 4".
    for (int targetY = 0; targetY < target_h; targetY++) {
        for (int targetX = 0; targetX < target_w; targetX++) {
            int ySampleStart = targetY * sample_rate;
            int ySampleEnd = ySampleStart + sample_rate;
            int xSampleStart = targetX * sample_rate;
            int xSampleEnd = xSampleStart + sample_rate;
            Color avgColor(0.0, 0.0, 0.0, 0.0);
            // Box sampling
            int samples = 0;
            for (int sampleY = ySampleStart; sampleY < ySampleEnd; sampleY++) {
                for (int sampleX = xSampleStart; sampleX < xSampleEnd; sampleX++) {
                    avgColor += sample_buffer[sampleY * sampleWidth + sampleX];
                    samples++;
                }
            }
            //cout << samples << "\n";
            avgColor *= (1.0 / (sample_rate * sample_rate));

            // Insert averaged color into rendering target
            render_target[4 * (targetX + targetY * target_w)] = (uint8_t)(avgColor.r * 255);
            render_target[4 * (targetX + targetY * target_w) + 1] = (uint8_t)(avgColor.g * 255);
            render_target[4 * (targetX + targetY * target_w) + 2] = (uint8_t)(avgColor.b * 255);
            render_target[4 * (targetX + targetY * target_w) + 3] = (uint8_t)(avgColor.a * 255);
        }
    }
    std::fill(sample_buffer.begin(), sample_buffer.end(), Color(1.0, 1.0, 1.0, 1.0));
}


} // namespace CMU462
