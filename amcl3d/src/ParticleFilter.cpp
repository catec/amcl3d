#include "ParticleFilter.h"

#include <gsl/gsl_randist.h>

namespace amcl3d
{
ParticleFilter::ParticleFilter()
{
  //! Setup random number generator from GSL
  gsl_rng_env_setup();
  random_value_ = gsl_rng_alloc(gsl_rng_default);
}

ParticleFilter::~ParticleFilter()
{
  gsl_rng_free(random_value_);
}

void ParticleFilter::buildParticlesPoseMsg(const geometry_msgs::Point32& offset, geometry_msgs::PoseArray& msg) const
{
  msg.poses.resize(p_.size());

  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    msg.poses[i].position.x = static_cast<double>(p_[i].x) + offset.x;
    msg.poses[i].position.y = static_cast<double>(p_[i].y) + offset.y;
    msg.poses[i].position.z = static_cast<double>(p_[i].z) + offset.z;
    msg.poses[i].orientation.x = 0.;
    msg.poses[i].orientation.y = 0.;
    msg.poses[i].orientation.z = sin(static_cast<double>(p_[i].a * 0.5f));
    msg.poses[i].orientation.w = cos(static_cast<double>(p_[i].a * 0.5f));
  }
}

void ParticleFilter::init(const int num_particles, const float x_init, const float y_init, const float z_init,
                          const float a_init, const float x_dev, const float y_dev, const float z_dev,
                          const float a_dev)
{
  //! Resize particle set
  p_.resize(abs(num_particles));

  //! Sample the given pose
  const float dev = std::max(std::max(x_dev, y_dev), z_dev);
  const float gaussConst1 = 1. / (dev * sqrt(2 * M_PI));
  const float gaussConst2 = 1. / (2 * dev * dev);

  p_[0].x = x_init;
  p_[0].y = y_init;
  p_[0].z = z_init;
  p_[0].a = a_init;
  p_[0].w = gaussConst1;

  float wt = p_[0].w;
  float dist;

  for (uint32_t i = 1; i < p_.size(); ++i)
  {
    p_[i].x = p_[0].x + gsl_ran_gaussian(random_value_, x_dev);
    p_[i].y = p_[0].y + gsl_ran_gaussian(random_value_, y_dev);
    p_[i].z = p_[0].z + gsl_ran_gaussian(random_value_, z_dev);
    p_[i].a = p_[0].a + gsl_ran_gaussian(random_value_, a_dev);

    dist = sqrt((p_[i].x - p_[0].x) * (p_[i].x - p_[0].x) + (p_[i].y - p_[0].y) * (p_[i].y - p_[0].y) +
                (p_[i].z - p_[0].z) * (p_[i].z - p_[0].z));

    p_[i].w = gaussConst1 * exp(-dist * dist * gaussConst2);

    wt += p_[i].w;
  }

  Particle mean_p;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    p_[i].w /= wt;

    mean_p.x += p_[i].w * p_[i].x;
    mean_p.y += p_[i].w * p_[i].y;
    mean_p.z += p_[i].w * p_[i].z;
    mean_p.a += p_[i].w * p_[i].a;
  }
  mean_ = mean_p;

  initialized_ = true;
}

void ParticleFilter::predict(const double odom_x_mod, const double odom_y_mod, const double odom_z_mod,
                             const double odom_a_mod, const double delta_x, const double delta_y, const double delta_z,
                             const double delta_a)
{
  const double x_dev = fabs(delta_x * odom_x_mod);
  const double y_dev = fabs(delta_y * odom_y_mod);
  const double z_dev = fabs(delta_z * odom_z_mod);
  const double a_dev = fabs(delta_a * odom_a_mod);

  //! Make a prediction for all particles according to the odometry
  float sa, ca, rand_x, rand_y;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    sa = sin(p_[i].a);
    ca = cos(p_[i].a);
    rand_x = delta_x + gsl_ran_gaussian(random_value_, x_dev);
    rand_y = delta_y + gsl_ran_gaussian(random_value_, y_dev);
    p_[i].x += ca * rand_x - sa * rand_y;
    p_[i].y += sa * rand_x + ca * rand_y;
    p_[i].z += delta_z + gsl_ran_gaussian(random_value_, z_dev);
    p_[i].a += delta_a + gsl_ran_gaussian(random_value_, a_dev);
  }
}

void ParticleFilter::update(const Grid3d& grid3d, const std::vector<pcl::PointXYZ>& points,
                            const std::vector<Range>& range_data, const double& alpha, const double& sigma_)
{
  //! Incorporate measurements
  float wtp = 0, wtr = 0;
  std::vector<pcl::PointXYZ> new_points;
  new_points.resize(points.size());

  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    //! Get particle information
    float tx = p_[i].x;
    float ty = p_[i].y;
    float tz = p_[i].z;
    float sa = sin(p_[i].a);
    float ca = cos(p_[i].a);

    //! Check the particle is into the map
    if (!grid3d.isIntoMap(tx, ty, tz))
    {
      // std::cout << "Not into map: " << grid3d_.isIntoMap(tx, ty, tz-1.0) << std::endl;
      p_[i].w = 0;
      continue;
    }

    //! Transform every point to current particle position
    for (uint32_t j = 0; j < points.size(); ++j)
    {
      //! Get point
      const pcl::PointXYZ& p = points[j];

      //! Translate and rotate it in yaw
      new_points[j].x = ca * p.x - sa * p.y + tx;
      new_points[j].y = sa * p.x + ca * p.y + ty;
      new_points[j].z = p.z + tz;
    }

    //! Evaluate the weight of the point cloud
    p_[i].wp = grid3d.computeCloudWeight(new_points);

    //! Evaluate the weight of the range sensors
    p_[i].wr = computeRangeWeight(tx, ty, tz, range_data, sigma_);

    //! Increase the summatory of weights
    wtp += p_[i].wp;
    wtr += p_[i].wr;
  }

  //! Normalize all weights
  float wt = 0;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    if (wtp > 0)
      p_[i].wp /= wtp;
    else
      p_[i].wp = 0;

    if (wtr > 0)
      p_[i].wr /= wtr;
    else
      p_[i].wr = 0;

    p_[i].w = p_[i].wp * alpha + p_[i].wr * (1 - alpha);
    wt += p_[i].w;
  }

  Particle mean_p;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    if (wt > 0)
      p_[i].w /= wt;
    else
      p_[i].w = 0;

    mean_p.x += p_[i].w * p_[i].x;
    mean_p.y += p_[i].w * p_[i].y;
    mean_p.z += p_[i].w * p_[i].z;
    mean_p.a += p_[i].w * p_[i].a;
  }
  mean_ = mean_p;
}

void ParticleFilter::resample()
{
  std::vector<Particle> newP(p_.size());
  const float factor = 1.f / p_.size();
  const float r = factor * gsl_rng_uniform(random_value_);
  float c = p_[0].w;
  float u;

  //! Do resamplig
  for (uint32_t m = 0, i = 0; m < p_.size(); ++m)
  {
    u = r + factor * m;
    while (u > c)
    {
      if (++i >= p_.size())
        break;
      c += p_[i].w;
    }
    newP[m] = p_[i];
    newP[m].w = factor;
  }

  //! Asign the new particles set
  p_ = newP;
}

float ParticleFilter::computeRangeWeight(const float x, const float y, const float z,
                                         const std::vector<Range>& range_data, const double sigma_)
{
  if (range_data.empty())
    return 0;

  float w = 1;
  const float k1 = 1.f / (sigma_ * sqrt(2 * M_PI));
  const float k2 = 0.5f / (sigma_ * sigma_);
  float ax, ay, az, r;
  for (uint32_t i = 0; i < range_data.size(); ++i)
  {
    ax = range_data[i].ax;
    ay = range_data[i].ay;
    az = range_data[i].az;
    r = sqrt((x - ax) * (x - ax) + (y - ay) * (y - ay) + (z - az) * (z - az));
    w *= k1 * exp(-k2 * (r - range_data[i].r) * (r - range_data[i].r));
  }

  return w;
}

}  // namespace amcl3d
