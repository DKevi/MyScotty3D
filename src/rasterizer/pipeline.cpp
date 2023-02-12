#include "pipeline.h"

#include "framebuffer.h"
#include "sample_pattern.h"

#include "../lib/log.h"
#include "../lib/mathlib.h"

#include <iostream>

template <PrimitiveType primitive_type, class Program, uint32_t flags>
void Pipeline<primitive_type, Program, flags>::run(
    std::vector<Vertex> const &vertices,
    typename Program::Parameters const &parameters,
    Framebuffer *framebuffer_)
{
  // Framebuffer must be non-null:
  assert(framebuffer_);
  auto &framebuffer = *framebuffer_;

  // A1T7: sample loop
  // TODO: update this function to rasterize to *all* sample locations in the framebuffer.
  //  This will probably involve inserting a loop of the form:
  //      std::vector< Vec3 > const &samples = framebuffer.sample_pattern.centers_and_weights;
  //      for (uint32_t s = 0; s < samples.size(); ++s) { ... }
  //   around some subset of the code.
  //  You will also need to transform the input and output of the rasterize_* functions to
  //    account for the fact they deal with pixels centered at (0.5,0.5).
  std::vector<ShadedVertex> shaded_vertices;
  shaded_vertices.reserve(vertices.size());

  //--------------------------
  // shade vertices:
  for (auto const &v : vertices)
  {
    ShadedVertex sv;
    Program::shade_vertex(parameters, v.attributes, &sv.clip_position, &sv.attributes);
    shaded_vertices.emplace_back(sv);
  }

  //--------------------------
  // assemble + clip + homogeneous divide vertices:
  std::vector<ClippedVertex> clipped_vertices;

  // reserve some space to avoid reallocations later:
  if constexpr (primitive_type == PrimitiveType::Lines)
  {
    // clipping lines can never produce more than one vertex per input vertex:
    clipped_vertices.reserve(shaded_vertices.size());
  }
  else if constexpr (primitive_type == PrimitiveType::Triangles)
  {
    // clipping triangles can produce up to 8 vertices per input vertex:
    clipped_vertices.reserve(shaded_vertices.size() * 8);
  }

  // coefficients to map from clip coordinates to framebuffer (i.e., "viewport") coordinates:
  // x: [-1,1] -> [0,width]
  // y: [-1,1] -> [0,height]
  // z: [-1,1] -> [0,1] (OpenGL-style depth range)
  Vec3 const clip_to_fb_scale = Vec3{
      framebuffer.width / 2.0f,
      framebuffer.height / 2.0f,
      0.5f};
  Vec3 const clip_to_fb_offset = Vec3{
      0.5f * framebuffer.width,
      0.5f * framebuffer.height,
      0.5f};

  // helper used to put output of clipping functions into clipped_vertices:
  auto emit_vertex = [&](ShadedVertex const &sv)
  {
    ClippedVertex cv;
    float inv_w = 1.0f / sv.clip_position.w;
    cv.fb_position = clip_to_fb_scale * inv_w * sv.clip_position.xyz() + clip_to_fb_offset;
    cv.inv_w = inv_w;
    cv.attributes = sv.attributes;
    clipped_vertices.emplace_back(cv);
  };

  // actually do clipping:
  if constexpr (primitive_type == PrimitiveType::Lines)
  {
    for (uint32_t i = 0; i + 1 < shaded_vertices.size(); i += 2)
    {
      clip_line(shaded_vertices[i], shaded_vertices[i + 1], emit_vertex);
    }
  }
  else if constexpr (primitive_type == PrimitiveType::Triangles)
  {
    for (uint32_t i = 0; i + 2 < shaded_vertices.size(); i += 3)
    {
      clip_triangle(shaded_vertices[i], shaded_vertices[i + 1], shaded_vertices[i + 2], emit_vertex);
    }
  }
  else
  {
    static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
  }

  //--------------------------
  // rasterize primitives:

  std::vector<Vec3> const &samples = framebuffer.sample_pattern.centers_and_weights;
  for (uint32_t s = 0; s < samples.size(); ++s)
  {
    std::vector<Fragment> fragments;
    float xShift, yShift;

    // helper used to put output of rasterization functions into fragments:
    auto emit_fragment = [&](Fragment const &f)
    {
      Fragment newF = f;
      newF.fb_position.x += xShift;
      newF.fb_position.y += yShift;
      fragments.emplace_back(newF);
    };
    xShift = samples[s].x - 0.5f;
    yShift = samples[s].y - 0.5f;
    // actually do rasterization:
    if constexpr (primitive_type == PrimitiveType::Lines)
    {
      for (uint32_t i = 0; i + 1 < clipped_vertices.size(); i += 2)
      {
        clipped_vertices[i].fb_position.x -= xShift;
        clipped_vertices[i].fb_position.y -= yShift;
        clipped_vertices[i + 1].fb_position.x -= xShift;
        clipped_vertices[i + 1].fb_position.y -= yShift;
        rasterize_line(clipped_vertices[i], clipped_vertices[i + 1], emit_fragment);
        clipped_vertices[i].fb_position.x += xShift;
        clipped_vertices[i].fb_position.y += yShift;
        clipped_vertices[i + 1].fb_position.x += xShift;
        clipped_vertices[i + 1].fb_position.y += yShift;
      }
    }
    else if constexpr (primitive_type == PrimitiveType::Triangles)
    {
      for (uint32_t i = 0; i + 2 < clipped_vertices.size(); i += 3)
      {
        clipped_vertices[i].fb_position.x -= xShift;
        clipped_vertices[i].fb_position.y -= yShift;
        clipped_vertices[i + 1].fb_position.x -= xShift;
        clipped_vertices[i + 1].fb_position.y -= yShift;
        clipped_vertices[i + 2].fb_position.x -= xShift;
        clipped_vertices[i + 2].fb_position.y -= yShift;
        rasterize_triangle(clipped_vertices[i], clipped_vertices[i + 1], clipped_vertices[i + 2], emit_fragment);
        clipped_vertices[i].fb_position.x += xShift;
        clipped_vertices[i].fb_position.y += yShift;
        clipped_vertices[i + 1].fb_position.x += xShift;
        clipped_vertices[i + 1].fb_position.y += yShift;
        clipped_vertices[i + 2].fb_position.x += xShift;
        clipped_vertices[i + 2].fb_position.y += yShift;
      }
    }
    else
    {
      static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
    }

    //--------------------------
    // depth test + shade + blend fragments:
    uint32_t out_of_range = 0; // check if rasterization produced fragments outside framebuffer (indicates something is wrong with clipping)
    for (auto const &f : fragments)
    {

      // fragment location (in pixels):
      int32_t x = (int32_t)std::floor(f.fb_position.x);
      int32_t y = (int32_t)std::floor(f.fb_position.y);

      // if clipping is working properly, this condition shouldn't be needed;
      // however, it prevents crashes while you are working on your clipping functions,
      // so we suggest leaving it in place:
      if (x < 0 || (uint32_t)x >= framebuffer.width || y < 0 || (uint32_t)y >= framebuffer.height)
      {
        ++out_of_range;
        continue;
      }

      // local names that refer to destination sample in framebuffer:
      float &fb_depth = framebuffer.depth_at(x, y, s);
      Spectrum &fb_color = framebuffer.color_at(x, y, s);

      // depth test:
      if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Always)
      {
        //"Always" means the depth test always passes.
      }
      else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Never)
      {
        //"Never" means the depth test never passes.
        continue; // discard this fragment
      }
      else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Less)
      {
        //"Less" means the depth test passes when the new fragment has depth less than the stored depth.
        // A1T4: Depth_Less
        // TODO: implement depth test! We want to only emit fragments that have a depth less than the stored depth, hence "Depth_Less"
        if (f.fb_position.z >= fb_depth)
          continue;
      }
      else
      {
        static_assert((flags & PipelineMask_Depth) <= Pipeline_Depth_Always, "Unknown depth test flag.");
      }

      // if depth test passes, and depth writes aren't disabled, write depth to depth buffer:
      if constexpr (!(flags & Pipeline_DepthWriteDisableBit))
      {
        fb_depth = f.fb_position.z;
      }

      // shade fragment:
      ShadedFragment sf;
      sf.fb_position = f.fb_position;
      Program::shade_fragment(parameters, f.attributes, f.derivatives, &sf.color, &sf.opacity);

      // write color to framebuffer if color writes aren't disabled:
      if constexpr (!(flags & Pipeline_ColorWriteDisableBit))
      {

        // blend fragment:
        if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Replace)
        {
          fb_color = sf.color;
        }
        else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Add)
        {
          // A1T4: Blend_Add
          // TODO: framebuffer color should have fragment color multiplied by fragment opacity added to it.
          fb_color += sf.opacity * sf.color; //<-- replace this line
        }
        else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Over)
        {
          // A1T4: Blend_Over
          // TODO: set framebuffer color to the result of "over" blending (also called "alpha blending") the fragment color over the framebuffer color, using the fragment's opacity
          //  		You may assume that the framebuffer color has its alpha premultiplied already, and you just want to compute the resulting composite color
          fb_color = sf.opacity * sf.color + (1 - sf.opacity) * fb_color; //<-- replace this line
        }
        else
        {
          static_assert((flags & PipelineMask_Blend) <= Pipeline_Blend_Over, "Unknown blending flag.");
        }
      }
    }
    if (out_of_range > 0)
    {
      if constexpr (primitive_type == PrimitiveType::Lines)
      {
        warn("Produced %d fragments outside framebuffer; this indicates something is likely wrong with the clip_line function.", out_of_range);
      }
      else if constexpr (primitive_type == PrimitiveType::Triangles)
      {
        warn("Produced %d fragments outside framebuffer; this indicates something is likely wrong with the clip_triangle function.", out_of_range);
      }
    }
  }
}

//-------------------------------------------------------------------------
// clipping functions

// helper to interpolate between vertices:
template <PrimitiveType p, class P, uint32_t F>
auto Pipeline<p, P, F>::lerp(ShadedVertex const &a, ShadedVertex const &b, float t) -> ShadedVertex
{
  ShadedVertex ret;
  ret.clip_position = (b.clip_position - a.clip_position) * t + a.clip_position;
  for (uint32_t i = 0; i < ret.attributes.size(); ++i)
  {
    ret.attributes[i] = (b.attributes[i] - a.attributes[i]) * t + a.attributes[i];
  }
  return ret;
}

/*
 * clip_line - clip line to portion with -w <= x,y,z <= w, emit vertices of clipped line (if non-empty)
 *  va, vb: endpoints of line
 *  emit_vertex: call to produce truncated line
 *
 * If clipping shortens the line, attributes of the shortened line should respect the pipeline's interpolation mode.
 *
 * If no portion of the line remains after clipping, emit_vertex will not be called.
 *
 * The clipped line should have the same direction as the full line.
 *
 */
template <PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_line(
    ShadedVertex const &va, ShadedVertex const &vb,
    std::function<void(ShadedVertex const &)> const &emit_vertex)
{
  // Determine portion of line over which:
  //  pt = (b-a) * t + a
  //  -pt.w <= pt.x <= pt.w
  //  -pt.w <= pt.y <= pt.w
  //  -pt.w <= pt.z <= pt.w

  //... as a range [min_t, max_t]:

  float min_t = 0.0f;
  float max_t = 1.0f;

  // want to set range of t for a bunch of equations like:
  //    a.x + t * ba.x <= a.w + t * ba.w
  // so here's a helper:
  auto clip_range = [&min_t, &max_t](float l, float dl, float r, float dr)
  {
    // restrict range such that:
    // l + t * dl <= r + t * dr
    // re-arranging:
    //  l - r <= t * (dr - dl)
    if (dr == dl)
    {
      // want: l - r <= 0
      if (l - r > 0.0f)
      {
        // works for none of range, so make range empty:
        min_t = 1.0f;
        max_t = 0.0f;
      }
    }
    else if (dr > dl)
    {
      // since dr - dl is positive:
      // want: (l - r) / (dr - dl) <= t
      min_t = std::max(min_t, (l - r) / (dr - dl));
    }
    else
    { // dr < dl
      // since dr - dl is negative:
      // want: (l - r) / (dr - dl) >= t
      max_t = std::min(max_t, (l - r) / (dr - dl));
    }
  };

  // local names for clip positions and their difference:
  Vec4 const &a = va.clip_position;
  Vec4 const &b = vb.clip_position;
  Vec4 const ba = b - a;

  // -a.w - t * ba.w <= a.x + t * ba.x <= a.w + t * ba.w
  clip_range(-a.w, -ba.w, a.x, ba.x);
  clip_range(a.x, ba.x, a.w, ba.w);
  // -a.w - t * ba.w <= a.y + t * ba.y <= a.w + t * ba.w
  clip_range(-a.w, -ba.w, a.y, ba.y);
  clip_range(a.y, ba.y, a.w, ba.w);
  // -a.w - t * ba.w <= a.z + t * ba.z <= a.w + t * ba.w
  clip_range(-a.w, -ba.w, a.z, ba.z);
  clip_range(a.z, ba.z, a.w, ba.w);

  if (min_t < max_t)
  {
    if (min_t == 0.0f)
    {
      emit_vertex(va);
    }
    else
    {
      ShadedVertex out = lerp(va, vb, min_t);
      // don't interpolate attributes if in flat shading mode:
      if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat)
        out.attributes = va.attributes;
      emit_vertex(out);
    }
    if (max_t == 1.0f)
    {
      emit_vertex(vb);
    }
    else
    {
      ShadedVertex out = lerp(va, vb, max_t);
      // don't interpolate attributes if in flat shading mode:
      if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat)
        out.attributes = va.attributes;
      emit_vertex(out);
    }
  }
}

/*
 * clip_triangle - clip triangle to portion with -w <= x,y,z <= w, emit resulting shape as triangles (if non-empty)
 *  va, vb, vc: vertices of triangle
 *  emit_vertex: call to produce clipped triangles (three calls per triangle)
 *
 * If clipping truncates the triangle, attributes of the new vertices should respect the pipeline's interpolation mode.
 *
 * If no portion of the triangle remains after clipping, emit_vertex will not be called.
 *
 * The clipped triangle(s) should have the same winding order as the full triangle.
 *
 */
template <PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_triangle(
    ShadedVertex const &va, ShadedVertex const &vb, ShadedVertex const &vc,
    std::function<void(ShadedVertex const &)> const &emit_vertex)
{
  // A1EC: clip_triangle
  // TODO: correct code!
  emit_vertex(va);
  emit_vertex(vb);
  emit_vertex(vc);
}

//-------------------------------------------------------------------------
// rasterization functions

/*
 * rasterize_line:
 * calls emit_fragment( frag ) for every pixel "covered" by the line (va.fb_position.xy, vb.fb_position.xy).
 *
 *    a pixel (x,y) is "covered" by the line if it exits the inscribed diamond:
 *
 *        (x+0.5,y+1)
 *        /        \
 *    (x,y+0.5)  (x+1,y+0.5)
 *        \        /
 *         (x+0.5,y)
 *
 *    to avoid ambiguity, we consider diamonds to contain their left and bottom points
 *    but not their top and right points.
 *
 * 	  since 45 degree lines breaks this rule, our rule in general is to rasterize the line as if its
 *    endpoints va and vb were at va + (e, e^2) and vb + (e, e^2) where no smaller nonzero e produces
 *    a different rasterization result.
 *
 * for each such diamond, pass Fragment frag to emit_fragment, with:
 *  - frag.fb_position.xy set to the center (x+0.5,y+0.5)
 *  - frag.fb_position.z interpolated linearly between va.fb_position.z and vb.fb_position.z
 *  - frag.attributes set to va.attributes (line will only be used in Interp_Flat mode)
 *  - frag.derivatives set to all (0,0)
 *
 * when interpolating the depth (z) for the fragments, you may use any depth the line takes within the pixel
 * (i.e., you don't need to interpolate to, say, the closest point to the pixel center)
 *
 * If you wish to work in fixed point, check framebuffer.h for useful information about the framebuffer's dimensions.
 *
 */

template <PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line(
    ClippedVertex const &va, ClippedVertex const &vb,
    std::function<void(Fragment const &)> const &emit_fragment)
{
  if constexpr ((flags & PipelineMask_Interp) != Pipeline_Interp_Flat)
  {
    assert(0 && "rasterize_line should only be invoked in flat interpolation mode.");
  }
  // A1T2: rasterize_line
  auto xmajor = [](ClippedVertex const &a, ClippedVertex const &b, ClippedVertex const &attributes, std::function<void(Fragment const &)> const &emit_fragment)
  {
    float dx, dy;
    dx = b.fb_position.x - a.fb_position.x;
    dy = b.fb_position.y - a.fb_position.y;

    float slope = dy / dx;
    for (float i = floor(a.fb_position.x) + 0.5f; i < floor(b.fb_position.x) + 0.5f; i = i + 1.0f)
    {
      // Set Fragment
      Fragment f;
      float y = a.fb_position.y + slope * (i - a.fb_position.x);
      f.fb_position = Vec3(i, round(y-0.5f) + 0.5f, a.fb_position.z + ((b.fb_position.z - a.fb_position.z) / (b.fb_position.x - a.fb_position.x) * (i - a.fb_position.x)));
      f.attributes = attributes.attributes;
      f.derivatives.fill(Vec2(0.0f, 0.0f));
      if (i != floor(a.fb_position.x) + 0.5f)
        emit_fragment(f);
    }
    return;
  };

  auto ymajor = [](ClippedVertex const &a, ClippedVertex const &b, ClippedVertex const &attributes, std::function<void(Fragment const &)> const &emit_fragment)
  {
    float dx, dy;
    dx = b.fb_position.x - a.fb_position.x;
    dy = b.fb_position.y - a.fb_position.y;

    float slope = dx / dy;
    for (float i = floor(a.fb_position.y) + 0.5f; i < floor(b.fb_position.y) + 0.5f; i = i + 1.0f)
    {
      // Set Fragment
      Fragment f;
      float x = a.fb_position.x + slope * (i - a.fb_position.y);
      f.fb_position = Vec3(round(x-0.5f) + 0.5f, i, a.fb_position.z + ((b.fb_position.z - a.fb_position.z) / (b.fb_position.y - a.fb_position.y) * (i - a.fb_position.y)));
      f.attributes = attributes.attributes;
      f.derivatives.fill(Vec2(0.0f, 0.0f));
      if (i != floor(a.fb_position.y) + 0.5f)
        emit_fragment(f);
    }
    return;
  };

  if (abs(vb.fb_position.y - va.fb_position.y) < abs(vb.fb_position.x - va.fb_position.x))
  {
    if (va.fb_position.x > vb.fb_position.x)
      xmajor(vb, va, va, emit_fragment);
    else
      xmajor(va, vb, va, emit_fragment);
  }
  else if (vb.fb_position.x != va.fb_position.x)
  {
    if (va.fb_position.y > vb.fb_position.y)
      ymajor(vb, va, va, emit_fragment);
    else
      ymajor(va, vb, va, emit_fragment);
  } else {
    // If vertical line
    // Step over Y
    float start = floor(va.fb_position.y) + 0.5f; // Start from va.y
    float end = floor(vb.fb_position.y) + 0.5f;   // End with vb.y
    float curr = floor(va.fb_position.x) + 0.5f;  // Changing X
    // The direction of the line
    float step;
    if (va.fb_position.y < vb.fb_position.y)
      step = 1.0f; // Upwards
    else
      step = -1.0f; // Downwards

    // Draw the line except start and end
    for (float i = start; i != end; i = i + step)
    {
      // Set Fragment
      Fragment f;
      f.fb_position = Vec3(curr, i, va.fb_position.z + ((vb.fb_position.z - va.fb_position.z) / (vb.fb_position.y - va.fb_position.y) * (i - va.fb_position.y)));
      f.attributes = va.attributes;
      f.derivatives.fill(Vec2(0.0f, 0.0f));
      // Draw the middle segment
      if (i != start)
        emit_fragment(f);
    }
  }

  // Diamond-exit
  // Start
  float xFloor, yFloor, dx, dy;
  xFloor = floor(va.fb_position.x);
  yFloor = floor(va.fb_position.y);
  dx = va.fb_position.x - xFloor;
  dy = va.fb_position.y - yFloor;
  if ((dy >= -dx + 0.5f) 
    && (dy < -dx + 1.5f) 
    && (dy > dx - 0.5f) 
    && (dy <= dx + 0.5f))
  {
    if (((vb.fb_position.y - yFloor) < -(vb.fb_position.x - xFloor) + 0.5f) 
      || ((vb.fb_position.y - yFloor) >= -(vb.fb_position.x - xFloor) + 1.5f) 
      || ((vb.fb_position.y - yFloor) <= vb.fb_position.x - xFloor - 0.5f) 
      || ((vb.fb_position.y - yFloor) > vb.fb_position.x - xFloor + 0.5f))
    {
      Fragment f;
      f.fb_position = Vec3(xFloor + 0.5f, yFloor + 0.5f, va.fb_position.z);
      f.attributes = va.attributes;
      f.derivatives.fill(Vec2(0.0f, 0.0f));
      emit_fragment(f);
    }
  } else if (((dy < -dx + 0.5f) 
              && (((vb.fb_position.y - yFloor) >= -(vb.fb_position.x - xFloor) + 1.5f) 
                || ((vb.fb_position.y >= yFloor) && (vb.fb_position.y - yFloor) <= vb.fb_position.x - xFloor - 0.5f) 
                || ((vb.fb_position.x >= xFloor) && (vb.fb_position.y - yFloor) > vb.fb_position.x - xFloor + 0.5f)))
          || ((dy >= -dx + 1.5f) 
              && (((vb.fb_position.y - yFloor) < -(vb.fb_position.x - xFloor) + 0.5f) 
                || ((vb.fb_position.y <= yFloor + 1) && (vb.fb_position.y - yFloor) <= vb.fb_position.x - xFloor - 0.5f) 
                || ((vb.fb_position.x <= xFloor + 1) && (vb.fb_position.y - yFloor) > vb.fb_position.x - xFloor + 0.5f))) 
          || ((dy <= dx - 0.5f) 
              && (((vb.fb_position.y - yFloor) > vb.fb_position.x - xFloor + 0.5f) 
                || ((vb.fb_position.x >= xFloor) && (vb.fb_position.y - yFloor) < -(vb.fb_position.x - xFloor) + 0.5f) 
                || ((vb.fb_position.y <= yFloor + 1) && (vb.fb_position.y - yFloor) >= -(vb.fb_position.x - xFloor) + 1.5f))) 
          || ((dy > dx + 0.5f) 
              && (((vb.fb_position.y - yFloor) <= vb.fb_position.x - xFloor - 0.5f)
                || ((vb.fb_position.y >= yFloor) && (vb.fb_position.y - yFloor) < -(vb.fb_position.x - xFloor) + 0.5f) 
                || ((vb.fb_position.x <= xFloor + 1) && (vb.fb_position.y - yFloor) >= -(vb.fb_position.x - xFloor) + 1.5f)))) {
    Fragment f;
    f.fb_position = Vec3(xFloor + 0.5f, yFloor + 0.5f, va.fb_position.z);
    f.attributes = va.attributes;
    f.derivatives.fill(Vec2(0.0f, 0.0f));
    emit_fragment(f);
  }
  // End
  xFloor = floor(vb.fb_position.x);
  yFloor = floor(vb.fb_position.y);
  dx = vb.fb_position.x - xFloor;
  dy = vb.fb_position.y - yFloor;
  if (((dy < -dx + 0.5f) 
        && (((va.fb_position.y - yFloor) >= -(va.fb_position.x - xFloor) + 1.5f) 
          || ((va.fb_position.y >= yFloor) && (va.fb_position.y - yFloor) <= va.fb_position.x - xFloor - 0.5f) 
          || ((va.fb_position.x >= xFloor) && (va.fb_position.y - yFloor) > va.fb_position.x - xFloor + 0.5f)))
      || ((dy >= -dx + 1.5f) 
        && (((va.fb_position.y - yFloor) < -(va.fb_position.x - xFloor) + 0.5f) 
          || ((va.fb_position.y >= yFloor) && (va.fb_position.y - yFloor) <= va.fb_position.x - xFloor - 0.5f) 
          || ((va.fb_position.x >= xFloor) && (va.fb_position.y - yFloor) > va.fb_position.x - xFloor + 0.5f))) 
      || ((dy <= dx - 0.5f) 
        && (((va.fb_position.y - yFloor) > va.fb_position.x - xFloor + 0.5f) 
          || ((va.fb_position.y >= yFloor) && (va.fb_position.y - yFloor) < -(va.fb_position.x - xFloor) + 0.5f) 
          || ((va.fb_position.x >= xFloor) && (va.fb_position.y - yFloor) >= -(va.fb_position.x - xFloor) + 1.5f))) 
      || ((dy > dx + 0.5f) 
        && (((va.fb_position.y - yFloor) <= va.fb_position.x - xFloor - 0.5f)
          || ((va.fb_position.y >= yFloor) && (va.fb_position.y - yFloor) < -(va.fb_position.x - xFloor) + 0.5f) 
          || ((va.fb_position.x >= xFloor) && (va.fb_position.y - yFloor) >= -(va.fb_position.x - xFloor) + 1.5f)))) {
    Fragment f;
    f.fb_position = Vec3(xFloor + 0.5f, yFloor + 0.5f, vb.fb_position.z);
    f.attributes = va.attributes;
    f.derivatives.fill(Vec2(0.0f, 0.0f));
    emit_fragment(f);
  }
}

/*
 *
 * rasterize_triangle(a,b,c,emit) calls 'emit(frag)' at every location
 *  (x+0.5,y+0.5) (where x,y are integers) covered by triangle (a,b,c).
 *
 * The emitted fragment should have:
 * - frag.fb_position.xy = (x+0.5, y+0.5)
 * - frag.fb_position.z = linearly interpolated fb_position.z from a,b,c (NOTE: does not depend on Interp mode!)
 * - frag.attributes = depends on Interp_* flag in flags:
 *   - if Interp_Flat: copy from va.attributes
 *   - if Interp_Screen: interpolate as if (a,b,c) is a 2D triangle flat on the screen
 *   - if Interp_Correct: use perspective-correct interpolation
 * - frag.derivatives = derivatives w.r.t. fb_position.x and fb_position.y of the first frag.derivatives.size() attributes.
 *
 * Notes on derivatives:
 *  The derivatives are partial derivatives w.r.t. screen locations. That is:
 *    derivatives[i].x = d/d(fb_position.x) attributes[i]
 *    derivatives[i].y = d/d(fb_position.y) attributes[i]
 *  You may compute these derivatives analytically or numerically.
 *
 *  See section 8.12.1 "Derivative Functions" of the GLSL 4.20 specification for some inspiration. (*HOWEVER*, the spec is solving a harder problem, and also nothing in the spec is binding on your implementation)
 *
 *  One approach is to rasterize blocks of four fragments and use forward and backward differences to compute derivatives.
 *  To assist you in this approach, keep in mind that the framebuffer size is *guaranteed* to be even. (see framebuffer.h)
 *
 * Notes on coverage:
 *  If two triangles are on opposite sides of the same edge, and a
 *  fragment center lies on that edge, rasterize_triangle should
 *  make sure that exactly one of the triangles emits that fragment.
 *  (Otherwise, speckles or cracks can appear in the final render.)
 *
 *  For degenerate (co-linear) triangles, you may consider them to not be on any side of an edge.
 * 	Thus, even if two degnerate triangles share an edge that contains a fragment center, you don't need to emit it.
 *  You will not lose points for doing something reasonable when handling this case
 *
 *  This is pretty tricky to get exactly right!
 *
 */
template <PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_triangle(
    ClippedVertex const &va, ClippedVertex const &vb, ClippedVertex const &vc,
    std::function<void(Fragment const &)> const &emit_fragment)
{
  auto TriArea = [](float ax, float ay, float bx, float by, float cx, float cy)
  {
    double abx = (double)bx - (double)ax;
    double aby = (double)by - (double)ay;
    double acx = (double)cx - (double)ax;
    double acy = (double)cy - (double)ay;
    return abs(abx*acy - aby*acx) / 2.0f;
  };

  auto TriAreaOrdered = [](float ax, float ay, float bx, float by, float cx, float cy)
  {
    double abx = (double)bx - (double)ax;
    double aby = (double)by - (double)ay;
    double acx = (double)cx - (double)ax;
    double acy = (double)cy - (double)ay;
    return (abx * acy - aby * acx) / 2.0f;
  };
  // Test edge va-vb is a left edge or top edge
  auto TestLeftOrTopEdge = [](ClippedVertex const &va, ClippedVertex const &vb, ClippedVertex const &vc)
  {
    // If edge is horizontal
    if (va.fb_position.y == vb.fb_position.y)
    {
      if (va.fb_position.y > vc.fb_position.y)
        return true;
      else
        return false;
    } else {
      float slope;
      if (va.fb_position.x < vb.fb_position.x) {
        slope = (vb.fb_position.y - va.fb_position.y) / (vb.fb_position.x - va.fb_position.x);
        if (slope > 0.0f) {
          if ((vc.fb_position.y - va.fb_position.y) < slope * (vc.fb_position.x - va.fb_position.x))
            return true;
          else 
            return false;
        } else {
          if ((vc.fb_position.y - va.fb_position.y) > slope * (vc.fb_position.x - va.fb_position.x))
            return true;
          else
            return false;
        }
      } else if (va.fb_position.x > vb.fb_position.x) {
        slope = (va.fb_position.y - vb.fb_position.y) / (va.fb_position.x - vb.fb_position.x);
        if (slope > 0.0f)
        {
          if ((vc.fb_position.y - vb.fb_position.y) < slope * (vc.fb_position.x - vb.fb_position.x))
            return true;
          else
            return false;
        }
        else
        {
          if ((vc.fb_position.y - vb.fb_position.y) > slope * (vc.fb_position.x - vb.fb_position.x))
            return true;
          else
            return false;
        }
      } else {
        if (vc.fb_position.x > va.fb_position.x)
          return true;
        else
          return false;
      }
    }
  };

  auto CalculateScreenSpaceDerivatives = [TriAreaOrdered](ClippedVertex const &va, ClippedVertex const &vb, ClippedVertex const &vc, double totalArea, Fragment &f, float deltaX, float deltaY)
  {
    double subArea1 = TriAreaOrdered(f.fb_position.x + deltaX, f.fb_position.y + deltaY, va.fb_position.x, va.fb_position.y, vb.fb_position.x, vb.fb_position.y);
    double subArea2 = TriAreaOrdered(f.fb_position.x + deltaX, f.fb_position.y + deltaY, vc.fb_position.x, vc.fb_position.y, va.fb_position.x, va.fb_position.y);
    double subArea3 = TriAreaOrdered(f.fb_position.x + deltaX, f.fb_position.y + deltaY, vb.fb_position.x, vb.fb_position.y, vc.fb_position.x, vc.fb_position.y);
    double u = subArea1 / totalArea;
    double v = subArea2 / totalArea;
    double w = subArea3 / totalArea;
    for (uint32_t i = 0; i < f.derivatives.size(); ++i)
    {
      float deltaAttribute = (float)((double)va.attributes[i] * w + (double)vb.attributes[i] * v + (double)vc.attributes[i] * u);
      if (deltaX != 0.0f)
      {
        if (deltaX > 0.0f)
          f.derivatives[i].x += deltaAttribute - f.attributes[i];
        else
          f.derivatives[i].x += f.attributes[i] - deltaAttribute;
      }
      else
      {
        if (deltaY > 0.0f)
          f.derivatives[i].y += deltaAttribute - f.attributes[i];
        else
          f.derivatives[i].y += f.attributes[i] - deltaAttribute;
      }
    }
    return;
  };

  auto CalculateCorrectDerivatives = [TriAreaOrdered](ClippedVertex const &va, ClippedVertex const &vb, ClippedVertex const &vc, double totalArea, Fragment &f, float deltaX, float deltaY)
  {
    double subArea1 = TriAreaOrdered(f.fb_position.x + deltaX, f.fb_position.y + deltaY, va.fb_position.x, va.fb_position.y, vb.fb_position.x, vb.fb_position.y);
    double subArea2 = TriAreaOrdered(f.fb_position.x + deltaX, f.fb_position.y + deltaY, vc.fb_position.x, vc.fb_position.y, va.fb_position.x, va.fb_position.y);
    double subArea3 = TriAreaOrdered(f.fb_position.x + deltaX, f.fb_position.y + deltaY, vb.fb_position.x, vb.fb_position.y, vc.fb_position.x, vc.fb_position.y);
    double u = subArea1 / totalArea;
    double v = subArea2 / totalArea;
    double w = subArea3 / totalArea;
    float fbw = (float)((double)va.inv_w * w + (double)vb.inv_w * v + (double)vc.inv_w * u);
    for (uint32_t i = 0; i < f.derivatives.size(); ++i)
    {
      float deltaAttribute = (float)(((double)va.attributes[i] * va.inv_w * w + (double)vb.attributes[i] * vb.inv_w * v + (double)vc.attributes[i] * vc.inv_w * u) / fbw);
      if (deltaX != 0.0f)
      {
        if (deltaX > 0.0f)
          f.derivatives[i].x += deltaAttribute - f.attributes[i];
        else
          f.derivatives[i].x += f.attributes[i] - deltaAttribute;
      }
      else
      {
        if (deltaY > 0.0f)
          f.derivatives[i].y += deltaAttribute - f.attributes[i];
        else
          f.derivatives[i].y += f.attributes[i] - deltaAttribute;
      }
    }
    return;
  };
  // NOTE: it is okay to restructure this function to allow these tasks to use the
  //  same code paths. Be aware, however, that all of them need to remain working!
  //  (e.g., if you break Flat while implementing Correct, you won't get points
  //   for Flat.)
  if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat)
  {
    // A1T3: flat triangles
    float bottomLeftX, bottomLeftY, topRightX, topRightY;
    // Find bottom left corner
    bottomLeftX = va.fb_position.x;
    if (vb.fb_position.x < bottomLeftX)
      bottomLeftX = vb.fb_position.x;
    if (vc.fb_position.x < bottomLeftX)
      bottomLeftX = vc.fb_position.x;
    bottomLeftY = va.fb_position.y;
    if (vb.fb_position.y < bottomLeftY)
      bottomLeftY = vb.fb_position.y;
    if (vc.fb_position.y < bottomLeftY)
      bottomLeftY = vc.fb_position.y;
    // Find topottom reft corner
    topRightX = va.fb_position.x;
    if (vb.fb_position.x > topRightX)
      topRightX = vb.fb_position.x;
    if (vc.fb_position.x > topRightX)
      topRightX = vc.fb_position.x;
    topRightY = va.fb_position.y;
    if (vb.fb_position.y > topRightY)
      topRightY = vb.fb_position.y;
    if (vc.fb_position.y > topRightY)
      topRightY = vc.fb_position.y;

    // Find centers
    bottomLeftX = floor(bottomLeftX) + 0.5f;
    bottomLeftY = floor(bottomLeftY) + 0.5f;
    topRightX = floor(topRightX) + 0.5f;
    topRightY = floor(topRightY) + 0.5f;

    double totalArea = TriArea(va.fb_position.x, va.fb_position.y, vb.fb_position.x, vb.fb_position.y, vc.fb_position.x, vc.fb_position.y);
    for (float i = bottomLeftX; i <= topRightX; i = i + 1.0f) 
    {
      for (float j = bottomLeftY; j <= topRightY; j = j + 1.0f)
      {
        double subArea1 = TriArea(va.fb_position.x, va.fb_position.y, vb.fb_position.x, vb.fb_position.y, i, j);
        double subArea2 = TriArea(va.fb_position.x, va.fb_position.y, vc.fb_position.x, vc.fb_position.y, i, j);
        double subArea3 = TriArea(vb.fb_position.x, vb.fb_position.y, vc.fb_position.x, vc.fb_position.y, i, j);
        float fbz = (float) ((double)va.fb_position.z * subArea3 / totalArea + (double)vb.fb_position.z * subArea2 / totalArea + (double) vc.fb_position.z * subArea1 / totalArea);
        if (subArea1 + subArea2 + subArea3 == totalArea)
        {
          if (subArea1 == 0.0f)
          {
            if (TestLeftOrTopEdge(va, vb, vc))
            {
              Fragment f;
              f.fb_position = Vec3(i, j, fbz);
              f.attributes = va.attributes;
              f.derivatives.fill(Vec2(0.0f, 0.0f));
              emit_fragment(f);
            }
          }
          else if (subArea2 == 0.0f)
          {
            if (TestLeftOrTopEdge(va, vc, vb))
            {
              Fragment f;
              f.fb_position = Vec3(i, j, fbz);
              f.attributes = va.attributes;
              f.derivatives.fill(Vec2(0.0f, 0.0f));
              emit_fragment(f);
            }
          }
          else if (subArea3 == 0.0f)
          {
            if (TestLeftOrTopEdge(vb, vc, va))
            {
              Fragment f;
              f.fb_position = Vec3(i, j, fbz);
              f.attributes = va.attributes;
              f.derivatives.fill(Vec2(0.0f, 0.0f));
              emit_fragment(f);
            }
          }
          else
          {
            Fragment f;
            f.fb_position = Vec3(i, j, fbz);
            f.attributes = va.attributes;
            f.derivatives.fill(Vec2(0.0f, 0.0f));
            emit_fragment(f);
          }
        }
      }
    }
  }
  else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Screen)
  {
    // A1T5: screen-space smooth triangles
    // TODO: rasterize triangle (see block comment above this function).
    float bottomLeftX, bottomLeftY, topRightX, topRightY;
    // Find bottom left corner
    bottomLeftX = va.fb_position.x;
    if (vb.fb_position.x < bottomLeftX)
      bottomLeftX = vb.fb_position.x;
    if (vc.fb_position.x < bottomLeftX)
      bottomLeftX = vc.fb_position.x;
    bottomLeftY = va.fb_position.y;
    if (vb.fb_position.y < bottomLeftY)
      bottomLeftY = vb.fb_position.y;
    if (vc.fb_position.y < bottomLeftY)
      bottomLeftY = vc.fb_position.y;
    // Find topottom reft corner
    topRightX = va.fb_position.x;
    if (vb.fb_position.x > topRightX)
      topRightX = vb.fb_position.x;
    if (vc.fb_position.x > topRightX)
      topRightX = vc.fb_position.x;
    topRightY = va.fb_position.y;
    if (vb.fb_position.y > topRightY)
      topRightY = vb.fb_position.y;
    if (vc.fb_position.y > topRightY)
      topRightY = vc.fb_position.y;

    // Find centers
    bottomLeftX = floor(bottomLeftX) + 0.5f;
    bottomLeftY = floor(bottomLeftY) + 0.5f;
    topRightX = floor(topRightX) + 0.5f;
    topRightY = floor(topRightY) + 0.5f;

    double totalArea = TriArea(va.fb_position.x, va.fb_position.y, vb.fb_position.x, vb.fb_position.y, vc.fb_position.x, vc.fb_position.y);
    for (float i = bottomLeftX; i <= topRightX; i = i + 1.0f)
    {
      for (float j = bottomLeftY; j <= topRightY; j = j + 1.0f)
      {
        double subArea1 = TriArea(va.fb_position.x, va.fb_position.y, vb.fb_position.x, vb.fb_position.y, i, j);
        double subArea2 = TriArea(va.fb_position.x, va.fb_position.y, vc.fb_position.x, vc.fb_position.y, i, j);
        double subArea3 = TriArea(vb.fb_position.x, vb.fb_position.y, vc.fb_position.x, vc.fb_position.y, i, j);
        double u = subArea1 / totalArea;
        double v = subArea2 / totalArea;
        double w = subArea3 / totalArea;
        if (subArea1 + subArea2 + subArea3 == totalArea)
        {
          Fragment f;
          float fbz = (float)((double)va.fb_position.z * w + (double)vb.fb_position.z * v + (double)vc.fb_position.z * u);
          for (uint32_t k = 0; k < f.attributes.size(); ++k)
          {
            f.attributes[k] = (float)((double)va.attributes[k] * w + (double)vb.attributes[k] * v + (double)vc.attributes[k] * u);
          }
          f.fb_position = Vec3(i, j, fbz);
          f.derivatives.fill(Vec2(0.0f, 0.0f));
          CalculateScreenSpaceDerivatives(va, vb, vc, totalArea, f, 1.0f, 0.0f);
          CalculateScreenSpaceDerivatives(va, vb, vc, totalArea, f, -1.0f, 0.0f);
          CalculateScreenSpaceDerivatives(va, vb, vc, totalArea, f, 0.0f, 1.0f);
          CalculateScreenSpaceDerivatives(va, vb, vc, totalArea, f, 0.0f, -1.0f);
          for (uint32_t k = 0; k < f.derivatives.size(); ++k)
          {
            f.derivatives[k].x = f.derivatives[k].x / 2.0f;
            f.derivatives[k].y = f.derivatives[k].y / 2.0f;
          }
          if (subArea1 == 0.0f) {
            if (TestLeftOrTopEdge(va, vb, vc))
              emit_fragment(f);
          }
          else if (subArea2 == 0.0f) {
            if (TestLeftOrTopEdge(va, vc, vb))
              emit_fragment(f);
          }
          else if (subArea3 == 0.0f) {
            if (TestLeftOrTopEdge(vb, vc, va))
              emit_fragment(f);
          }
          else
            emit_fragment(f);
        }
      }
    }
  }
  else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Correct)
  {
    // A1T5: perspective correct triangles
    // TODO: rasterize triangle (block comment above this function).
    float bottomLeftX, bottomLeftY, topRightX, topRightY;
    // Find bottom left corner
    bottomLeftX = va.fb_position.x;
    if (vb.fb_position.x < bottomLeftX)
      bottomLeftX = vb.fb_position.x;
    if (vc.fb_position.x < bottomLeftX)
      bottomLeftX = vc.fb_position.x;
    bottomLeftY = va.fb_position.y;
    if (vb.fb_position.y < bottomLeftY)
      bottomLeftY = vb.fb_position.y;
    if (vc.fb_position.y < bottomLeftY)
      bottomLeftY = vc.fb_position.y;
    // Find topottom reft corner
    topRightX = va.fb_position.x;
    if (vb.fb_position.x > topRightX)
      topRightX = vb.fb_position.x;
    if (vc.fb_position.x > topRightX)
      topRightX = vc.fb_position.x;
    topRightY = va.fb_position.y;
    if (vb.fb_position.y > topRightY)
      topRightY = vb.fb_position.y;
    if (vc.fb_position.y > topRightY)
      topRightY = vc.fb_position.y;

    // Find centers
    bottomLeftX = floor(bottomLeftX) + 0.5f;
    bottomLeftY = floor(bottomLeftY) + 0.5f;
    topRightX = floor(topRightX) + 0.5f;
    topRightY = floor(topRightY) + 0.5f;

    double totalArea = TriArea(va.fb_position.x, va.fb_position.y, vb.fb_position.x, vb.fb_position.y, vc.fb_position.x, vc.fb_position.y);
    for (float i = bottomLeftX; i <= topRightX; i = i + 1.0f)
    {
      for (float j = bottomLeftY; j <= topRightY; j = j + 1.0f)
      {
        double subArea1 = TriArea(va.fb_position.x, va.fb_position.y, vb.fb_position.x, vb.fb_position.y, i, j);
        double subArea2 = TriArea(va.fb_position.x, va.fb_position.y, vc.fb_position.x, vc.fb_position.y, i, j);
        double subArea3 = TriArea(vb.fb_position.x, vb.fb_position.y, vc.fb_position.x, vc.fb_position.y, i, j);
        double u = subArea1 / totalArea;
        double v = subArea2 / totalArea;
        double w = subArea3 / totalArea;
        if (subArea1 + subArea2 + subArea3 == totalArea)
        {
          Fragment f;
          float fbz = (float)((double)va.fb_position.z * w + (double)vb.fb_position.z * v + (double)vc.fb_position.z * u);
          float fbw = (float)((double)va.inv_w * w + (double)vb.inv_w * v + (double)vc.inv_w * u);
          for (uint32_t k = 0; k < f.attributes.size(); ++k)
          {
            f.attributes[k] = (float)(((double)va.attributes[k] * va.inv_w * w + (double)vb.attributes[k] * vb.inv_w * v + (double)vc.attributes[k] * vc.inv_w * u) / fbw);
          }
          f.fb_position = Vec3(i, j, fbz);
          f.derivatives.fill(Vec2(0.0f, 0.0f));
          CalculateCorrectDerivatives(va, vb, vc, totalArea, f, 1.0f, 0.0f);
          CalculateCorrectDerivatives(va, vb, vc, totalArea, f, -1.0f, 0.0f);
          CalculateCorrectDerivatives(va, vb, vc, totalArea, f, 0.0f, 1.0f);
          CalculateCorrectDerivatives(va, vb, vc, totalArea, f, 0.0f, -1.0f);
          for (uint32_t k = 0; k < f.derivatives.size(); ++k)
          {
            f.derivatives[k].x = f.derivatives[k].x / 2.0f;
            f.derivatives[k].y = f.derivatives[k].y / 2.0f;
          }
          if (subArea1 == 0.0f)
          {
            if (TestLeftOrTopEdge(va, vb, vc))
              emit_fragment(f);
          }
          else if (subArea2 == 0.0f)
          {
            if (TestLeftOrTopEdge(va, vc, vb))
              emit_fragment(f);
          }
          else if (subArea3 == 0.0f)
          {
            if (TestLeftOrTopEdge(vb, vc, va))
              emit_fragment(f);
          }
          else
            emit_fragment(f);
        }
      }
    }
  }
}

//-------------------------------------------------------------------------
// compile instantiations for all programs and blending and testing types:

#include "programs.h"

template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian, Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian, Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian, Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian, Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Screen>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian, Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
