#include "ee4308_drone/estimator.hpp"

namespace ee4308::drone
{
    Estimator::Estimator(
        const rclcpp::NodeOptions &options,
        const std::string &name = "estimator")
        : Node(name, options)
    {
        // parameters
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 10.0).as_double();
        this->var_imu_x_ = ee4308::getParameter<double>(this, "var_imu_x", 0.2).as_double();
        this->var_imu_y_ = ee4308::getParameter<double>(this, "var_imu_y", 0.2).as_double();
        this->var_imu_z_ = ee4308::getParameter<double>(this, "var_imu_z", 0.2).as_double();
        this->var_imu_a_ = ee4308::getParameter<double>(this, "var_imu_a", 0.2).as_double();
        this->var_gps_x_ = ee4308::getParameter<double>(this, "var_gps_x", 0.2).as_double();
        this->var_gps_y_ = ee4308::getParameter<double>(this, "var_gps_y", 0.2).as_double();
        this->var_gps_z_ = ee4308::getParameter<double>(this, "var_gps_z", 0.2).as_double();
        this->var_baro_ = ee4308::getParameter<double>(this, "var_baro", 0.2).as_double();
        this->var_sonar_ = ee4308::getParameter<double>(this, "var_sonar", 0.2).as_double();
        this->var_magnet_ = ee4308::getParameter<double>(this, "var_magnet", 0.2).as_double();
        this->verbose_ = ee4308::getParameter<bool>(this, "verbose", true).as_bool();
        this->frame_id_map_ = ee4308::getParameter<std::string>(this, "map_frame_id", "map").as_string();
        this->frame_id_drone_ = ee4308::getParameter<std::string>(this, "drone_frame_id", "drone/base_link").as_string();
        this->use_ground_truth_ = ee4308::getParameter<bool>(this, "use_ground_truth", false).as_bool();
        double initial_x = ee4308::getParameter<double>(this, "initial_x", -2.0).as_double();
        double initial_y = ee4308::getParameter<double>(this, "initial_y", -2.0).as_double();
        double initial_z = ee4308::getParameter<double>(this, "initial_z", 0.05).as_double();
        gps_file_.open("gps_data.csv");
        sonar_file_.open("sonar_data.csv");
        magnet_file_.open("magnet_data.csv");
        imu_file_.open("imu_data.csv");
        biasfile.open("biasfile.csv");
        gps_file_ << "time,x,y,z\n";
        sonar_file_ << "time,range\n";
        magnet_file_ << "time,heading\n";
        imu_file_ << "time,x,y,z\n" ;
        biasfile << "time,gpsx,gpsy,z\n" ;
        // we maintain an average understanding of X and Y velocities in order to detect when we are static. 

        

        
        ///// MAKE SURE THIS NEXT VARIABLE IS IINITIALISED IN THE HEADER FILE
        // tuning flags to prevent re-calculations




        // topics
        this->pub_est_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::ServicesQoS());
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1); // use only the most recent
        this->sub_true_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "true_odom", qos, std::bind(&Estimator::callbackSubTrueOdom_, this, std::placeholders::_1)); // ground truth in sim.
        this->sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "fix", qos, std::bind(&Estimator::callbackSubGPS_, this, std::placeholders::_1));
        this->sub_sonar_ = this->create_subscription<sensor_msgs::msg::LaserScan>( // gz has no sonar implementation. laserscan for quick hack.
            "sonar", qos, std::bind(&Estimator::callbackSubSonar_, this, std::placeholders::_1));
        this->sub_magnetic_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "magnetic", qos, std::bind(&Estimator::callbackSubMagnetic_, this, std::placeholders::_1));
        this->sub_baro_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "air_pressure", qos, std::bind(&Estimator::callbackSubBaro_, this, std::placeholders::_1));
        this->sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", qos, std::bind(&Estimator::callbackSubIMU_, this, std::placeholders::_1));

        // states
        this->initial_position_ << initial_x, initial_y, initial_z;
        this->Xx_ << initial_x, 0,0.08;
        this->Xy_ << initial_y, 0,-0.08;
        this->Xz_ << initial_z, 0,0.035;
        this->Xa_ << 0, 0;
        this->Px_ = Eigen::Matrix3d::Constant(1e3),
        this->Py_ = Eigen::Matrix3d::Constant(1e3),
        this->Pz_ = Eigen::Matrix3d::Constant(1e3);
        this->Pa_ = Eigen::Matrix2d::Constant(1e3);
        this->initial_ECEF_ << NAN, NAN, NAN;
        this->Ygps_ << NAN, NAN, NAN;
        this->Ymagnet_ = NAN;
        this->Ybaro_ = NAN;
        this->Ysonar_ = NAN;

        this->last_predict_time_ = this->now().seconds();
        this->initialized_ecef_ = false;
        this->initialized_magnetic_ = false;

        this->timer_ = this->create_timer(
            1s / this->frequency_,
            std::bind(&Estimator::callbackTimer, this));
    }

    // ================================ GPS sub callback / EKF Correction ========================================
    Eigen::Vector3d Estimator::getECEF_(
        const double &sin_lat, const double &cos_lat,
        const double &sin_lon, const double &cos_lon,
        const double &alt)
    {
        Eigen::Vector3d ECEF;
        // ==== make use of ====
        // RAD_POLAR, RAD_EQUATOR
        // std::sqrt()
        // all the function arguments.
        // =========
        double esq = 1- (std::pow(RAD_POLAR/RAD_EQUATOR,2));
        double N = RAD_EQUATOR / std::sqrt(1-esq*sin_lat*sin_lat);
        // rewrite or delete the following
        ECEF << (N+alt)*cos_lat*cos_lon, (N+alt)*cos_lat*sin_lon, ((1-esq)*N+alt)*sin_lat;
        return ECEF;
    }

    void Estimator::callbackSubGPS_(const sensor_msgs::msg::NavSatFix msg)
    { // avoiding const & due to possibly long calcs.
        (void)msg;

        constexpr double DEG2RAD = M_PI / 180;
        double lat = msg.latitude * DEG2RAD;  
        double lon = msg.longitude * DEG2RAD; 
        double alt = msg.altitude;

        double sin_lat = sin(lat);
        double cos_lat = cos(lat);
        double sin_lon = sin(lon);
        double cos_lon = cos(lon);

        if (initialized_ecef_ == false)
        {
            initial_ECEF_ = getECEF_(sin_lat, cos_lat, sin_lon, cos_lon, alt);
            sinlat = sin_lat;
            coslat = cos_lat;
            sinlon = sin_lon;
            coslon = cos_lon;
            refalt = alt;
            initialized_ecef_ = true;
            return;
        }
        Eigen::Vector3d ECEF = getECEF_(sin_lat, cos_lat, sin_lon, cos_lon, alt);
        Eigen::MatrixXd Ren;
        Ren.resize(3,3);
        Ren << -sinlat*coslon, -sinlon, -coslat*coslon, -sinlat*sinlon, coslon, -coslat*sinlon, coslat, 0, -sinlat;
        Eigen::Vector3d NED  = Ren.transpose()*(ECEF-initial_ECEF_);
        Eigen::MatrixXd Rmn ;
        Rmn.resize(3,3);
        Rmn <<  0 , 1 , 0 , 1 , 0 , 0 , 0 , 0 , -1;
        Ygps_ =  Rmn*NED + initial_position_;
        
        
        Eigen::MatrixXd Hx;
        Hx.resize(1,3);
        Hx << 1,0,1;
        Eigen::MatrixXd Hy;
        Hy.resize(1,3);
        Hy << 1,0,1;
        double time = this->now().seconds();

        if (gps_file_.is_open()) {
            gps_file_ << time << ","
                    << Ygps_(0) << ","
                    << Ygps_(1) << ","
                    << Ygps_(2) << "\n";
        }
        var_gps_x_= 0.009873171;
        var_gps_y_= 0.012809;
        var_gps_z_= 0.0051;
        Eigen::MatrixXd Rx;
        Rx.resize(1,1);
        Rx(0,0)=var_gps_x_;
        Eigen::MatrixXd Ry;
        Ry.resize(1,1);
        Ry(0,0)=var_gps_y_;
        Eigen::MatrixXd Rz;
        Rz.resize(1,1);
        Rz(0,0)=var_gps_z_;
        double xresid = (Ygps_(0) - (Hx*Xx_)(0));///(6.568*0.113);
        double yresid = (Ygps_(1) - (Hx*Xy_)(0));///(6.568*0.113);
        
        // SEMI TUNED TUKEY WEIGHTING
         // Essentially a filter to reject outlying values, focussed more on precision than accuracy.
        //if (std::abs(xresid)>1){
//            xresid=xresid*0.5;
  //      } else { xresid=xresid*(1-xresid*xresid)*(1-xresid*xresid);}
    //    if (std::abs(yresid)>1){
      //      yresid=yresid*0.5;
        //} else { yresid=yresid*(1-yresid*yresid)*(1-yresid*yresid);}
        // THESE are the measurements we process in order to calculate our covariances
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::MatrixXd Hz;
        Hz.resize(1,3);
        Hz<<1,0,1;
        Eigen::MatrixXd Kx = Px_*Hx.transpose()*(Hx*Px_*Hx.transpose() + Rx).inverse();
        Eigen::MatrixXd Ky = Py_*Hy.transpose()*(Hy*Py_*Hy.transpose() + Ry).inverse();
        
        Eigen::MatrixXd Kz = Pz_*Hz.transpose()*(Hz*Pz_*Hz.transpose() + Rz).inverse();
        //double xresid = (Ygps_(0) - (Hx*Xx_)(0))/(6.568*0.113);
        //double yresid = (Ygps_(1) - (Hy*Xy_)(0))/(6.568*0.113);
//        if (std::abs(xresid/(0.113*4.568))>1){
  //          xresid=0;
    //    } else { xresid=xresid/(0.113*4.568)*(1-(xresid/(0.113*4.568))*(xresid/(0.113*4.568)))*(1-(xresid/(0.113*4.568))*(xresid/(0.113*4.568)));}
      //  if (std::abs(yresid/(0.113*4.568))>1){
        //    yresid=0;
       // } else { yresid=yresid/(0.113*4.568)*(1-(yresid/(0.113*4.568))*(yresid/(0.113*4.568)))*(1-(yresid/(0.113*4.568))*(yresid/(0.113*4.568)));}
        Xx_ = Xx_ + Kx*(xresid);
        Xy_ = Xy_ + Ky*(yresid);
        Xz_ = Xz_ + Kz*(Ygps_(2) - (Hz*Xz_)(0));
        //  if (biasfile.is_open()) {
  //          biasfile << time << ","
    //                << Xx_(0) << ","
      //              << Xy_(0) << "\n";
        //}
     //   if (Xx_(1)<=0.35){
       //     maxX=-100000;
         //   minX=10000;
      
        // }
        Xvel.push_front(Xx_(1));
        Yvel.push_front(Xy_(1));
        if (Xvel.size()>15){
            Xvel.pop_back();
        }
        if (Yvel.size()>15){
            Yvel.pop_back();
        }
        double avgy=0;
        double avgx=0;
        for(double v: Xvel){
            avgx+=v;
        }
        for(double v: Yvel){
            avgy+=v;
        }
        avgy=avgy/15;
        avgx=avgx/15;
        
        
        Eigen::MatrixXd Ix = I-Kx*Hx;
        Eigen::MatrixXd Iy = I-Ky*Hy;
        Px_ = (Ix)*Px_*(Ix).transpose()+Kx*Rx*Kx.transpose();
        Py_ = (Iy)*Py_*(Iy).transpose()+Ky*Ry*Ky.transpose();
        Pz_ = Pz_ - Kz*Hz*Pz_;
        Px_=(Px_+Px_.transpose())/2;
        Py_=(Py_+Py_.transpose())/2;
        Pz_=(Pz_+Pz_.transpose())/2;
        if(biasfile.is_open()){
            biasfile << time << "," << Xx_(0) << "," << Xy_(0) << "," << Xz_(0) << "\n" ;
        }
        if (imu_file_.is_open()) {
            imu_file_ << time << "," << Xx_(0) << "," << Xy_(0) <<"\n";
        }
    }

    // ================================ Sonar sub callback / EKF Correction ========================================
    void Estimator::callbackSubSonar_(const sensor_msgs::msg::LaserScan msg)
    {
        // Store the measured sonar range in Ysonar_.
        //      Required for terminal printing during demonstration.
        // ==== make use of ====
        // msg.ranges[0]
        // Ysonar_
        // var_sonar_
        // Xz_
        // Pz_
        // .transpose()
        // =========
        if(msg.ranges.empty()){
            Ysonar_=NAN;
            return;
        }
        Ysonar_ = msg.ranges[0];
        double time = this->now().seconds();

        if (std::isfinite(Ysonar_) && sonar_file_.is_open()) {
            sonar_file_ << time << "," << Ysonar_ << "\n";
        }
 
        if (!std::isfinite(Ysonar_))
        { 
            // if out of range, write to Ysonar_, 
            //     but do not do the KF correction.
            return;
        }

        // ==== [FOR LAB 2 ONLY] ==== 
        // The following is necessary so that the covariance bubble in RViz does not fill up the screen.
        // For proj 2, comment out the following:
        //Px_ << 0.1, 0, 0, 0.1;
        //Py_ << 0.1, 0, 0, 0.1;
        // =========
        var_sonar_=0.00868;
        Eigen::MatrixXd H ;
        H.resize(1,3);
        H << 1,0,1;
        Eigen::MatrixXd R ;
        R.resize(1,1);
        R(0,0)=var_sonar_;
        Eigen::MatrixXd K = Pz_*H.transpose()*((H*Pz_*H.transpose() + R).inverse());
        Xz_=Xz_+K*(Ysonar_-(H*Xz_)(0,0));
        Pz_=Pz_-K*H*Pz_;
        Pz_=(Pz_+Pz_.transpose())/2;
        if(biasfile.is_open()){
            biasfile << time << "," << Xx_(0) << "," << Xy_(0) << "," << Xz_(0) << "\n" ;
        }

        
        
        // if in range, write to Ysonar_, and do the KF correction.
    }

    // ================================ Magnetic sub callback / EKF Correction ========================================
    void Estimator::callbackSubMagnetic_(const sensor_msgs::msg::MagneticField msg)
    {
        // Store the measured angle (world frame) in Ymagnet_.
        //      Required for terminal printing during demonstration.
        // Along the horizontal plane, the magnetic north in Gazebo points towards +x, when it should point to +y (even if world is configured to be ENU frame). It is a bug.
        // As the drone always starts pointing towards +x, there is no need to offset the calculation with an initial heading.
        // Magnetic force direction in drone's z-axis can be ignored.
        // The units are in Gauss (by Gazebo) instead of in Tesla (MagneticField message documentation).
        // ==== make use of ====
        // Ymagnet_
        // msg.magnetic_field.x // the magnetic force direction along drone's x-axis.
        // msg.magnetic_field.y // the magnetic force direction along drone's y-axis.
        // std::atan2()
        // Xa_
        // Pa_
        // var_magnet_
        // .transpose()
        // limitAngle()
        // =======
        Ymagnet_ = limitAngle(std::atan2(msg.magnetic_field.y,msg.magnetic_field.x));
        double time = this->now().seconds();

        if (magnet_file_.is_open()) {
            magnet_file_ << time << "," << Ymagnet_ << "\n";
        }
        //if (Magnest==0){
    //        if(Magnet.size()<100){
      //          Magnet.push_front(Ymagnet_);
        //    }
          //  double avg=0;
            //for(double v: Magnet){
//                avg+=v;
  //          }
    //        avg=avg/100;
      //      double cov = 0 ;
        //    for(double a : Magnet){
          //      cov+=std::pow(a-avg,2);
            //}
 //           var_magnet_=cov;
   //         Magnest=1;
     //   }
        var_magnet_=0.00573;
        Eigen::MatrixXd Ha;
        Ha.resize(1,2);
        Ha << 1,0;
        Eigen::MatrixXd R;
        R.resize(1,1);
        R<<var_magnet_;
        
        Eigen::MatrixXd K = Pa_*Ha.transpose()*(Ha*Pa_*Ha.transpose()+R).inverse();

        R<<Ymagnet_; // we just reuse this variable to implement our correction
        Xa_=Xa_+K*limitAngle(-Ymagnet_-(Ha*Xa_)(0,0));
        Pa_ = Pa_ - K*Ha*Pa_;
        Pa_ = (Pa_+Pa_.transpose())/2;
        
        // rewrite or delete the following:
    }

    // ================================ Baro sub callback / EKF Correction ========================================
    void Estimator::callbackSubBaro_(const sensor_msgs::msg::FluidPressure msg)
    {
        // Store the measured barometer altitude in Ybaro_.
        //      Required for terminal printing during demonstration.
        // the fluid pressure is in pascal.
        // ==== make use of ====
        // Ybaro_ 
        // SEA_LEVEL_PA
        // msg.fluid_pressure
        // var_baro_
        // Pz_
        // Xz_
        // .transpose()
        // =========

        rclcpp::Time tnow = msg.header.stamp;
        double dt = tnow.seconds() - last_predict_time_;
        last_predict_time_ = tnow.seconds();
        double time = this->now().seconds();

        
        Ybaro_= 44330*(1-std::pow(msg.fluid_pressure/SEA_LEVEL_PA,0.1903));
        if (magnet_file_.is_open()) {
            magnet_file_ << time << "," << Ybaro_ << "\n";
        }
        Eigen::MatrixXd Hb;
        Hb.resize(1,3);
        Hb << 1,0,1;
        Eigen::MatrixXd Rb;
        Rb.resize(1,1);
        var_baro_ = 0.000523064551601084;
        Rb << var_baro_;
        Eigen::MatrixXd Kb = Pz_*Hb.transpose()*(Hb*Pz_*Hb.transpose()+Rb).inverse();
        Xz_ = Xz_+Kb*(Ybaro_-(Hb*Xz_)(0,0));
        Pz_ = Pz_ - Kb*Hb*Pz_;
        Pz_ = (Pz_ + Pz_.transpose())/2;
        if(biasfile.is_open()){
            biasfile << time << "," << Xx_(0) << "," << Xy_(0) << "," << Xz_(0) << "\n" ;
        }
        // rewrite or delete the following
        (void) msg;
    }

    // ================================ IMU sub callback / EKF Prediction ========================================
    void Estimator::callbackSubIMU_(const sensor_msgs::msg::Imu msg)
    {
        rclcpp::Time tnow = msg.header.stamp;
        double dt = tnow.seconds() - last_predict_time_;
        last_predict_time_ = tnow.seconds();
        double time = this->now().seconds();
        
        
        if (dt < ee4308::THRES)
            return;
        var_imu_x_= 0.127193001680166;
        var_imu_y_= 0.102602855859187;
        var_imu_z_=	0.00444166643018195;
        var_imu_a_ = 0.00912250642704444 ;//* 0.00912250642704444;
        Eigen::MatrixXd F;
        F.resize(3,3);
        F << 1, dt, 0, 0, 1, 0, 0, 0, 1;
        Eigen::Vector2d Wa;
        Eigen::Vector3d Wz;
        Eigen::MatrixXd Wx;
        Eigen::MatrixXd Wy;
        Wz(0,0)=0.5*dt*dt;
        Wz(1,0)=dt;
        Wz(2,0)=0;
        Wa(0,0)=dt;
        Wa(1,0)=1;
        Wx.resize(3,2);
        Wy.resize(3,2);
        Wx(0,0)=0.5*dt*dt*std::cos(Xa_(0));
        Wx(0,1)=-0.5*dt*dt*std::sin(Xa_(0));
        Wx(1,0)=dt*std::cos(Xa_(0));
        Wx(1,1)=-dt*std::sin(Xa_(0));
        Wx(2,0)=0;
        Wx(2,1)=0;
        Wy(0,0)=0.5*dt*dt*std::sin(Xa_(0));
        Wy(0,1)=0.5*dt*dt*std::cos(Xa_(0));
        Wy(1,0)=dt*std::sin(Xa_(0));
        Wy(1,1)=dt*std::cos(Xa_(0));
        Wy(2,0)=0;
        Wy(2,1)=0;
        Eigen::MatrixXd Qax;
        Qax.resize(2,2);
        Qax(0,0)=var_imu_x_;
        Qax(0,1)=0;
        Qax(1,0)=0;
        Qax(1,1) = var_imu_y_;
        
        Eigen::MatrixXd Fa;
        Fa.resize(2,2);
        Fa(0,0)=1;
        Fa(1,0)=0;
        Fa(0,1)=0;
        Fa(1,1)=0;

        Eigen::Vector2d Ux;
        Ux << msg.linear_acceleration.x , msg.linear_acceleration.y;
        Xx_=F*Xx_+Wx*Ux;
        Xy_=F*Xy_+Wy*Ux;
        Eigen::MatrixXd Fz;
        Fz.resize(3,3);
        Fz << 1, dt, 0, 0, 1, 0, 0, 0, 1;
        Xz_=Fz*Xz_+Wz*(msg.linear_acceleration.z-GRAVITY);
        Xa_ = Fa*Xa_+Wa*msg.angular_velocity.z;// Xa_(0)+(Xa_(1) + msg.angular_velocity.z)*dt/2, msg.angular_velocity.z;
        Px_=F*Px_*F.transpose()+Wx*Qax*Wx.transpose();
        Py_=F*Py_*F.transpose()+Wy*Qax*Wy.transpose();
        Pz_=Fz*Pz_*Fz.transpose()+Wz*var_imu_z_*Wz.transpose();
        Pa_=Fa*Pa_*Fa.transpose()+Wa*var_imu_a_*Wa.transpose(); // VERY DOUBTFUL, THIS LINE
        Px_=(Px_+Px_.transpose())/2;
        Py_=(Py_+Py_.transpose())/2;
        Pz_=(Pz_+Pz_.transpose())/2;
        Pa_=(Pa_+Pa_.transpose())/2;
        if(biasfile.is_open()){
            biasfile << time << "," << Xx_(0) << "," << Xx_(1) << Xy_(0) << "," << Xy_(1) << "\n" ;
        }

        // NOT ALLOWED TO USE ORIENTATION FROM IMU as ORIENTATION IS DERIVED FROM ANGULAR VELOCTIY !!!
        // Store the states in Xx_, Xy_, Xz_, and Xa_ for terminal printing.
        // Store the covariances in Px_, Py_, Pz_, and Pa_ for terminal printing.
        // ==== make use of ====
        // msg.linear_acceleration
        // msg.angular_velocity
        // GRAVITY
        // var_imu_x_, var_imu_y_, var_imu_z_, var_imu_a_
        // Xx_, Xy_, Xz_, Xa_
        // Px_, Py_, Pz_, Pa_
        // dt
        // std::cos(), std::sin()
        // =========

        // rewrite or delete the following
        (void) msg;
    }

    void Estimator::callbackSubTrueOdom_(const nav_msgs::msg::Odometry msg)
    {
        this->true_odom_ = msg;
    }

    void Estimator::callbackTimer()
    {
        // ======= Publish and Verbose Ground Truth =======
        if (this->use_ground_truth_)
        {
            this->pub_est_odom_->publish(this->true_odom_);

            if (this->verbose_)
            {
                double t = this->now().seconds();
                std::stringstream ss;
                ss << std::fixed;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "TruPose" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.x << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.y << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.z << "\t"
                   << std::setw(7) << std::setprecision(3) << ee4308::getYawFromQuaternion(true_odom_.pose.pose.orientation)
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "TruTwis" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.x << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.y << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.z << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.angular.z
                   << std::endl;
                std::cout << ss.str() << std::endl;
            }
        }
        // ======= Publish and Verbose KF =======
        else
        {
            // you can extend this to include velocities if you want, but the topic name may have to change from pose to something else.
            // odom is already taken.
            nav_msgs::msg::Odometry odom;

            odom.header.stamp = this->now();
            odom.child_frame_id = "";     //; std::string(this->get_namespace()) + "/base_footprint";
            odom.header.frame_id = "map"; //; std::string(this->get_namespace()) + "/odom";

            odom.pose.pose.position.x = Xx_[0];
            odom.pose.pose.position.y = Xy_[0];
            odom.pose.pose.position.z = Xz_[0];
            getQuaternionFromYaw(Xa_[0], odom.pose.pose.orientation);
            odom.pose.covariance[0] = Px_(0, 0);
            odom.pose.covariance[7] = Py_(0, 0);
            odom.pose.covariance[14] = Pz_(0, 0);
            odom.pose.covariance[35] = Pa_(0, 0);

            odom.twist.twist.linear.x = Xx_[1];
            odom.twist.twist.linear.y = Xy_[1];
            odom.twist.twist.linear.z = Xz_[1];
            odom.twist.twist.angular.z = Xa_[1];
            odom.twist.covariance[0] = Px_(1, 1);
            odom.twist.covariance[7] = Py_(1, 1);
            odom.twist.covariance[14] = Pz_(1, 1);
            odom.twist.covariance[35] = Pa_(1, 1);


            odom.pose.pose.position.y = Xy_(0);


            pub_est_odom_->publish(odom);

            if (verbose_)
            {
                double t = this->now().seconds();
                std::stringstream ss;
                ss << std::fixed;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Pose" << "\t"
                   << std::setw(7) << std::setprecision(3) << Xx_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xy_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xz_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << ee4308::limitAngle(Xa_(0))
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Twist" << "\t"
                   << std::setw(7) << std::setprecision(3) << Xx_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xy_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xz_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Xa_(1)
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "ErrPose" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.x - Xx_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.y - Xy_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.pose.pose.position.z - Xz_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << ee4308::limitAngle(ee4308::getYawFromQuaternion(true_odom_.pose.pose.orientation) - Xa_(0))
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "ErrTwis" << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.x - Xx_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.y - Xy_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.linear.z - Xz_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << true_odom_.twist.twist.angular.z - Xa_(1)
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "GPS" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ygps_(0) << "\t"
                   << std::setw(7) << std::setprecision(3) << Ygps_(1) << "\t"
                   << std::setw(7) << std::setprecision(3) << Ygps_(2) << "\t"
                   << std::setw(7) << "--"
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Baro"<< "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ybaro_ << "\t"
                   << std::setw(7) << "--"
                   << std::endl;
                // ss << "\t"
                //    << std::setw(7) << std::setprecision(3) << t << "\t"
                //    << std::setw(7) << "BBias"<< "\t"
                //    << std::setw(7) << "--" << "\t"
                //    << std::setw(7) << "--" << "\t"
                //    << std::setw(7) << std::setprecision(3) << Xz_(2) << "\t"
                //    << std::setw(7) << "--"
                //    << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Sonar"<< "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ysonar_ << "\t"
                   << std::setw(7) << "--"
                   << std::endl;
                ss << "\t"
                   << std::setw(7) << std::setprecision(3) << t << "\t"
                   << std::setw(7) << "Magnt"<< "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << "--" << "\t"
                   << std::setw(7) << std::setprecision(3) << Ymagnet_
                   << std::endl;
                std::cout << ss.str() << std::endl;
            }
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Estimator);