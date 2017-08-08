 

   class OctreeCentroidContainer : public pcl::octree::OctreeContainerBase
    {
      public:
        /** \brief Class initialization. */
      OctreeCentroidContainer ()
        {
          this->reset();
        }

        /** \brief Empty class deconstructor. */
        virtual ~OctreeCentroidContainer ()
        {
        }

        /** \brief deep copy function */
        virtual OctreeCentroidContainer *
        deepCopy () const
        {
          return (new OctreeCentroidContainer (*this));
        }

        /** \brief Add new point to voxel.
          * \param[in] new_point the new point to add
          */
        void
        addPoint (const pcl::PointXYZRGB& new_point, unsigned int
time_stamp)
        {
          ++point_counter_;

          x_sum_ += new_point.x;
          y_sum_ += new_point.y;
          z_sum_ += new_point.z;

          color_r_ += new_point.r;
          color_g_ += new_point.g;
          color_b_ += new_point.b;

          time_stamp_ = time_stamp;
        }

        /** \brief Calculate centroid of voxel.
          * \param[out] centroid_arg the resultant centroid of the voxel
          */
        void
        getCentroid (pcl::PointXYZRGB& centroid_arg) const
        {
          if (point_counter_)
          {
            float fc = static_cast<float> (point_counter_);
            centroid_arg.x = x_sum_ / fc;
            centroid_arg.y = y_sum_ / fc;
            centroid_arg.z = z_sum_ / fc;

            centroid_arg.r = color_r_ / fc;
            centroid_arg.g = color_g_ / fc;
            centroid_arg.b = color_b_ / fc;
          }
        }

        /** \brief Calculate centroid of voxel.
          * \param[out] centroid_arg the resultant centroid of the voxel
          */
        unsigned int
        getTimeStamp () const
        {
          return time_stamp_;
        }

        /** \brief Reset leaf container. */
        virtual void
        reset ()
        {
          point_counter_ = 0;

          color_r_ = 0.0;
          color_g_ = 0.0;
          color_b_ = 0.0;

          x_sum_ = y_sum_ = z_sum_ = 0.0;

          time_stamp_ = 0;

        }

      private:
        unsigned int point_counter_;

        unsigned int color_r_;
        unsigned int color_g_;
        unsigned int color_b_;

        float x_sum_;
        float y_sum_;
        float z_sum_;

        unsigned int time_stamp_;
    }; 
