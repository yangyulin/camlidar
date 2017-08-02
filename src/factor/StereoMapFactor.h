//
// Created by linde on 7/31/17.
//

#ifndef FACTORTEST_STEREOMAPFACTOR_H
#define FACTORTEST_STEREOMAPFACTOR_H

/**
 * @file    StereoMapFactor.h
 * @brief   A non-linear factor for stereo measurements
 * @author  Yulin Yang
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/StereoCamera.h>

namespace gtsam {

/**
 * A Generic Stereo Factor
 * @addtogroup SLAM
 */
    class StereoMapFactor: public NoiseModelFactor1<Pose3> {
    private:

        // Keep a copy of measurement and calibration for I/O
        StereoPoint2 measured_;                       ///< the measurement
        Cal3_S2Stereo::shared_ptr K_;                ///< shared pointer to calibration
        Pose3 T_B_S_;                               ///< The pose of the sensor in the body frame
        Point3 P_G_f_;                               ///< the feature point in the global frame

        // verbosity handling for Cheirality Exceptions
        //bool throwCheirality_;                       ///< If true, rethrows Cheirality exceptions (default: false)
        //bool verboseCheirality_;                     ///< If true, prints text for Cheirality exceptions (default: false)

    public:

        // shorthand for base class type
        typedef NoiseModelFactor1<Pose3> Base;              ///< typedef for base class
        typedef StereoMapFactor<Pose3> This;                ///< typedef for this class (with templates)
        typedef boost::shared_ptr<StereoMapFactor> shared_ptr;  ///< typedef for shared pointer to this object
        //typedef POSE CamPose;                                       ///< typedef for Pose Lie Value type

        /**
         * Default constructor
         */
        StereoMapFactor() : K_(new Cal3_S2Stereo(444, 555, 666, 777, 888, 1.0)) {}

        /**
         * Constructor
         * @param measured is the Stereo Point measurement (u_l, u_r, v). v will be identical for left & right for rectified stereo pair
         * @param model is the noise model in on the measurement
         * @param poseKey the pose variable key
         * @param K the constant calibration
         * @param T_B_S is the transform from body to sensor frame (default identity)
         * @param P_G_f is the feature point in the global frame
         */
        StereoMapFactor(const StereoPoint2& measured, const SharedNoiseModel& model,
                            Key T_G_B_Key,  const Cal3_S2Stereo::shared_ptr& K,
                            Pose3 T_B_S, Point3 P_G_f) :
                Base(model, T_G_B_Key), measured_(measured), K_(K), T_B_S_(T_B_S), P_G_f_(P_G_f) {}


        /** Virtual destructor */
        virtual ~StereoMapFactor() {}

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

        /**
         * print
         * @param s optional string naming the factor
         * @param keyFormatter optional formatter useful for printing Symbols
         */
        void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
            Base::print(s, keyFormatter);
            measured_.print(s + ".z");
            if(this->T_B_S_)
                this->T_B_S_->print("  sensor pose in body frame: ");
            if(this->P_G_f_)
                this->P_G_f_->print("  Feature point in the global frame: ");
        }

        /**
         * equals
         */
        virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
            const StereoMapFactor* e = dynamic_cast<const StereoMapFactor*> (&f);
            return e
                   && Base::equals(f)
                   && measured_.equals(e->measured_, tol)
                   && T_B_S_.equals(e->T_B_S_, tol)
                   && P_G_f_.equals(e->P_G_f_, tol);
        }

        /** h(x)-z */
        Vector evaluateError(const Pose3& T_G_B,
                             boost::optional<Matrix&> H_G_B = boost::none) const {

            ///compute the estimated stereopoints
            gtsam::Rot3 R_G_B = T_G_B.rotation();
            gtsam::Point3 P_G_B = T_G_B.translation();
            gtsam::Rot3 R_B_S = T_B_S_.rotation();
            gtsam::Point3 P_B_S = T_B_S_.translation();

            gtsam::Point3 P_B_f = R_G_B.transpose() * (P_G_f_ - P_G_B);
            gtsam::Point3 P_S_f = R_B_S.transpose() * (P_B_f - P_B_S);

            // readout the calibration parameter
            double fx, fy, px, py, b;
            fx = K_->fx(); fy = K_->fy(); px = K_->px(); py = K_->py(); b = K_->baseline();

            double u1_norm = P_S_f.x()/P_S_f.z();
            double u2_norm = (P_S_f.x() - b)/P_S_f.z();
            double v1_norm = P_S_f.y()/P_S_f.z();

            gtsam::Point3 P_normal(u1_norm, u2_norm, v1_norm);

            double u1 = u1_norm * fx + px;
            double u2 = u2_norm * fx + px;
            double v1 = v1_norm * fy + py;

            gtsam::StereoPoint2 P_hat_img(u1, u2, v1);

            gtsam::Matrix3 dimg_dnorm;
            dimg_dnorm << fx, 0, 0,
                          0, fx, 0,
                          0, 0, fy;

            gtsam::Matrix3 dnorm_dP_S_f;
            dnorm_dP_S_f << 1/P_normal.z(),          0,        -P_normal.x()/P_normal.z()/P_normal.z(),
                            1/P_normal.z(),          0,        -(P_normal.x()-b)/P_normal.z()/P_normal.z(),
                            0,              1/P_normal.z(),    -P_normal.y()/P_normal.z()/P_normal.z();

            gtsam::Matrix3 dimg_dP_S_f = dimg_dnorm * dnorm_dP_S_f;

            gtsam::Matrix3 dP_S_f_dR_G_B = R_B_S.transpose().matrix() * skewSymmetric(P_B_f.vector());
            gtsam::Matrix3 dP_S_f_dP_G_B = -(R_G_B * R_B_S).transpose().matrix();
            gtsam::Matrix36 dP_S_f_dT_G_B;
            dP_S_f_dT_G_B << dP_S_f_dR_G_B, dP_S_f_dP_G_B;

            gtsam::Matrix36 dimg_dT_G_B;
            dimg_dT_G_B = dimg_dP_S_f * dP_S_f_dT_G_B;

            if (H_G_B) *H_G_B = dimg_dT_G_B;

            return (P_hat_img - measured_).vector();

        }

        /** return the measured */
        const StereoPoint2& measured() const {
            return measured_;
        }

        /** return the calibration object */
        inline const Cal3_S2Stereo::shared_ptr calibration() const {
            return K_;
        }


    private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive & ar, const unsigned int /*version*/) {
            ar & boost::serialization::make_nvp("NoiseModelFactor1",
                                                boost::serialization::base_object<Base>(*this));
            ar & BOOST_SERIALIZATION_NVP(measured_);
            ar & BOOST_SERIALIZATION_NVP(K_);
            ar & BOOST_SERIALIZATION_NVP(T_B_S_);
            ar & BOOST_SERIALIZATION_NVP(P_G_f_);
        }
    };

/// traits
    template<class T1, class T2>
    struct traits<StereoMapFactor<T1> > : public Testable<StereoMapFactor<T1> > {};

} // \ namespace gtsam



#endif //FACTORTEST_STEREOMAPFACTOR_H
