
#include "samplers.h"
#include "../util/rand.h"

namespace Samplers
{

	Vec2 Rect::sample(RNG &rng) const
	{
		// A3T1 - step 2 - supersampling

		// Return a point selected uniformly at random from the rectangle [0,size.x)x[0,size.y)
		// Useful function: rng.unit()
		float randx = size.x * rng.unit();
		float randy = size.y * rng.unit();

		return Vec2{randx, randy};
	}

	float Rect::pdf(Vec2 at) const
	{
		if (at.x < 0.0f || at.x > size.x || at.y < 0.0f || at.y > size.y)
			return 0.0f;
		return 1.0f / (size.x * size.y);
	}

	Vec3 Point::sample(RNG &rng) const
	{
		return point;
	}

	float Point::pdf(Vec3 at) const
	{
		return at == point ? 1.0f : 0.0f;
	}

	Vec3 Triangle::sample(RNG &rng) const
	{
		float u = std::sqrt(rng.unit());
		float v = rng.unit();
		float a = u * (1.0f - v);
		float b = u * v;
		return a * v0 + b * v1 + (1.0f - a - b) * v2;
	}

	float Triangle::pdf(Vec3 at) const
	{
		float a = 0.5f * cross(v1 - v0, v2 - v0).norm();
		float u = 0.5f * cross(at - v1, at - v2).norm() / a;
		float v = 0.5f * cross(at - v2, at - v0).norm() / a;
		float w = 1.0f - u - v;
		if (u < 0.0f || v < 0.0f || w < 0.0f)
			return 0.0f;
		if (u > 1.0f || v > 1.0f || w > 1.0f)
			return 0.0f;
		return 1.0f / a;
	}

	Vec3 Hemisphere::Uniform::sample(RNG &rng) const
	{

		float Xi1 = rng.unit();
		float Xi2 = rng.unit();

		float theta = std::acos(Xi1);
		float phi = 2.0f * PI_F * Xi2;

		float xs = std::sin(theta) * std::cos(phi);
		float ys = std::cos(theta);
		float zs = std::sin(theta) * std::sin(phi);

		return Vec3(xs, ys, zs);
	}

	float Hemisphere::Uniform::pdf(Vec3 dir) const
	{
		if (dir.y < 0.0f)
			return 0.0f;
		return 1.0f / (2.0f * PI_F);
	}

	Vec3 Hemisphere::Cosine::sample(RNG &rng) const
	{

		float phi = rng.unit() * 2.0f * PI_F;
		float cos_t = std::sqrt(rng.unit());

		float sin_t = std::sqrt(1 - cos_t * cos_t);
		float x = std::cos(phi) * sin_t;
		float z = std::sin(phi) * sin_t;
		float y = cos_t;

		return Vec3(x, y, z);
	}

	float Hemisphere::Cosine::pdf(Vec3 dir) const
	{
		if (dir.y < 0.0f)
			return 0.0f;
		return dir.y / PI_F;
	}

	Vec3 Sphere::Uniform::sample(RNG &rng) const
	{
		// A3T7 - sphere sampler

		// Generate a uniformly random point on the unit sphere.
		// Tip: start with Hemisphere::Uniform
		float phi = rng.unit() * 2.0f * PI_F;
		float theta = rng.unit() * PI_F;
		float cos_theta = std::cos(theta);
		float sin_theta = std::sqrt(1 - cos_theta * cos_theta);

		float xs = std::cos(phi) * sin_theta;
		float ys = std::sin(phi) * sin_theta;
		float zs = cos_theta;

		return Vec3(xs, ys, zs);
	}

	float Sphere::Uniform::pdf(Vec3 dir) const
	{
		return 1.0f / (4.0f * PI_F);
	}

	Sphere::Image::Image(const HDR_Image &image)
	{
		// A3T7 - image sampler init

		// Set up importance sampling data structures for a spherical environment map image.
		// You may make use of the _pdf, _cdf, and total members, or create your own.

		const auto [_w, _h] = image.dimension();
		w = _w;
		h = _h;
		float pdf_sum = 0.0f;
		for (uint32_t i = 0; i < h; i++)
		{
			for (uint32_t j = 0; j < w; j++)
			{
				float theta = PI_F * (float(i) + 0.5f) / float(h);
				float luma = image.at(j + i * w).luma();
				float local_pdf = std::sin(theta) * luma;
				pdf_sum += local_pdf;
				_pdf.push_back(local_pdf);
			}
		}
		float cdf_sum = 0.0f;
		for (uint32_t i = 0; i < h; i++)
		{
			for (uint32_t j = 0; j < w; j++)
			{
				_pdf[j + i * w] /= pdf_sum;
				cdf_sum += _pdf[j + i * w];
				_cdf.push_back(cdf_sum);
			}
		}
	}

	Vec3 Sphere::Image::sample(RNG &rng) const
	{
		// A3T7 - image sampler sampling

		// Use your importance sampling data structure to generate a sample direction.
		// Tip: std::upper_bound
		auto prob = std::upper_bound(_cdf.begin(), _cdf.end(), rng.unit());
		if (prob == _cdf.end())
			prob -= 1;

		int idx = int(prob - _cdf.begin());

		int y = idx / (int)w;
		int x = idx - w * y;
		float phi = (float(x) + 0.5f) / float(w) * 2.0f * PI_F;
		float theta = (float(y) + 0.5f) / float(h) * PI_F;

		float xs = std::sin(theta) * std::cos(phi);
		float ys = -std::cos(theta);
		float zs = std::sin(theta) * std::sin(phi);

		return Vec3(xs, ys, zs);
	}

	float Sphere::Image::pdf(Vec3 dir) const
	{
		// A3T7 - image sampler pdf

		// What is the PDF of this distribution at a particular direction?
		float u = std::atan2(dir.z, dir.x) / (2.0f * PI_F);
		if (u < 0.0f)
			u += 1.0f;
		float v = std::acos(-1.0f * std::clamp(dir.y, -1.0f, 1.0f)) / PI_F;

		uint32_t x = (uint32_t(u * w) == w) ? w - 1 : uint32_t(u * w);

		uint32_t y = (uint32_t(h * v) == h) ? h - 1 : uint32_t(h * v);

		uint32_t idx = x + y * w;

		float theta = std::acos(-dir.y);

		float jacobian = float(w) * float(h) / (2 * PI_F * PI_F * std::sin(theta));
		return jacobian * _pdf[idx];
	}

} // namespace Samplers
