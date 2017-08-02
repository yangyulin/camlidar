//
// Created by linde on 7/31/17.
//

#ifndef FACTORTEST_STEREOCALIBMAPFACTOR_H
#define FACTORTEST_STEREOCALIBMAPFACTOR_H

/**
 * @file    StereoCalibMapFactor.h
 * @brief   A non-linear factor for stereo measurements
 * @author  Yulin Yang
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>
#include <boost/optional.hpp>

namespace gtsam {

/**
 * A Generic Stereo Factor
 * @addtogroup SLAM
 */
    class GTSAM_EXPORT StereoCalibMapFactor: public NoiseModelFactor2<Pose3, Pose3> {
    private:

        // Keep a copy of measurement and calibration for I/O
        StereoPoint2 measured_;                       ///< the measurement
        Cal3_S2Stereo::shared_ptr K_;                ///< shared pointer to calibration
        //Pose3 T_B_S_;                               ///< The pose of the sensor in the body frame
        Point3 P_G_f_;                               ///< the feature point in the global frame

        // verbosity handling for Cheirality Exceptions
        //bool throwCheirality_;                       ///< If true, rethrows Cheirality exceptions (default: false)
        //bool verboseCheirality_;                     ///< If true, prints text for Cheirality exceptions (default: false)

    public:

        // shorthand for base class type
        typedef NoiseModelFactor2<Pose3, Pose3> Base;              ///< typedef for base class
        typedef StereoCalibMapFactor This;                ///< typedef for this class (with templates)
        typedef boost::shared_ptr<StereoCalibMapFactor> shared_ptr;  ///< typedef for shared pointer to this object
        //typedef POSE CamPose;                                       ///< typedef for Pose Lie Value type

        /**
         * Default constructor
         */
        StereoCalibMapFactor() : K_(new Cal3_S2Stereo(444, 555, 666, 777, 888, 1.0)) {}

        /**
         * Constructor
         * @param measured is the Stereo Point measurement (u_l, u_r, v). v will be identical for left & right for rectified stereo pair
         * @param model is the noise model in on the measurement
         * @param poseKey the pose variable key
         * @param K the constant calibration
         * @param P_G_f is the feature point in the global frame
         */
        StereoCalibMapFactor(const StereoPoint2& measured,
                             const SharedNoiseModel& model,
                             Key T_G_B_Key,
                             Key T_B_S_Key,
                             const Cal3_S2Stereo::shared_ptr& K,
                            Point3 P_G_f) :
                Base(model, T_G_B_Key, T_B_S_Key), measured_(measured), K_(K), P_G_f_(P_G_f) {}


        /** Virtual destructor */
        virtual ~StereoCalibMapFactor() {}

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
            P_G_f_.print("  Feature point in the global frame: ");
        }

        /**
         * equals
         */
        virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
            const StereoCalibMapFactor * e = dynamic_cast<const StereoCalibMapFactor *> (&f);
            return e
                   && Base::equals(f)
                   && measured_.equals(e->measured_, tol)
                   && P_G_f_.equals(e->P_G_f_, tol);
        }

        /** h(x)-z */
        Vector evaluateError(const Pose3& T_G_B,
                             const Pose3& T_B_S,
                             boost::optional<Matrix&> H_G_B = boost::none,
                             boost::optional<Matrix&> H_B_S = boost::none) const {

            ///compute the estimated stereopoints
            gtsam::Rot3 R_G_B = T_G_B.rotation();
            gtsam::Point3 P_G_B = T_G_B.translation();
            gtsam::Rot3 R_B_S = T_B_S.rotation();
            gtsam::Point3 P_B_S = T_B_S.translation();

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

            gtsam::Matrix3 dP_S_f_dR_B_S = skewSymmetric(P_S_f.vector());
            gtsam::Matrix3 dP_S_f_dP_B_S = -(R_B_S).transpose().matrix();
            gtsam::Matrix36 dP_S_f_dT_B_S;
            dP_S_f_dT_B_S << dP_S_f_dR_B_S, dP_S_f_dP_B_S;
            gtsam::Matrix36 dimg_dT_B_S;
            dimg_dT_B_S = dimg_dP_S_f * dP_S_f_dT_B_S;

            if (H_G_B) *H_G_B = dimg_dT_G_B;
            if (H_B_S) *H_B_S = dimg_dT_B_S;

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
            ar & boost::serialization::make_nvp("NoiseModelFactor2",
                                                boost::serialization::base_object<Base>(*this));
            ar & BOOST_SERIALIZATION_NVP(measured_);
            ar & BOOST_SERIALIZATION_NVP(K_);
            ar & BOOST_SERIALIZATION_NVP(P_G_f_);
        }
    };

/// traits
//    template<class T1, class T2>
//    struct traits<StereoCalibMapFactor<T1> > : public Testable<StereoCalibMapFactor<T1> > {};

    /**
    * A Generic Stereo Factor for NavState
    * @addtogroup SLAM
    */
    class StereoCalibMapFactor2: public NoiseModelFactor2<NavState, Pose3> {
    private:

        // Keep a copy of measurement and calibration for I/O
        StereoPoint2 measured_;                       ///< the measurement
        Cal3_S2Stereo::shared_ptr K_;                ///< shared pointer to calibration
        //Pose3 T_B_S_;                               ///< The pose of the sensor in the body frame
        Point3 P_G_f_;                               ///< the feature point in the global frame

        // verbosity handling for Cheirality Exceptions
        //bool throwCheirality_;                       ///< If true, rethrows Cheirality exceptions (default: false)
        //bool verboseCheirality_;                     ///< If true, prints text for Cheirality exceptions (default: false)

    public:

        // shorthand for base class type
        typedef NoiseModelFactor2<NavState, Pose3> Base;              ///< typedef for base class
        typedef StereoCalibMapFactor2 This;                ///< typedef for this class (with templates)
        typedef boost::shared_ptr<StereoCalibMapFactor2> shared_ptr;  ///< typedef for shared pointer to this object
        //typedef POSE CamPose;                                       ///< typedef for Pose Lie Value type

        /**
         * Default constructor
         */
        StereoCalibMapFactor2() : K_(new Cal3_S2Stereo(444, 555, 666, 777, 888, 1.0)) {}

        /**
         * Constructor
         * @param measured is the Stereo Point measurement (u_l, u_r, v). v will be identical for left & right for rectified stereo pair
         * @param model is the noise model in on the measurement
         * @param poseKey the pose variable key
         * @param K the constant calibration
         * @param P_G_f is the feature point in the global frame
         */
        StereoCalibMapFactor2(const StereoPoint2& measured,
                             const SharedNoiseModel& model,
                             Key TN_G_B_Key,
                             Key T_B_S_Key,
                             const Cal3_S2Stereo::shared_ptr& K,
                             Point3 P_G_f) :
                Base(model, TN_G_B_Key, T_B_S_Key), measured_(measured), K_(K), P_G_f_(P_G_f) {}


        /** Virtual destructor */
        virtual ~StereoCalibMapFactor2() {}

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
            P_G_f_.print("  Feature point in the global frame: ");
        }

        /**
         * equals
         */
        virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
            const StereoCalibMapFactor2* e = dynamic_cast<const StereoCalibMapFactor2*> (&f);
            return e
                   && Base::equals(f)
                   && measured_.equals(e->measured_, tol)
                   && P_G_f_.equals(e->P_G_f_, tol);
        }

        /** h(x)-z */
        Vector evaluateError(const NavState& TN_G_B,
                             const Pose3& T_B_S,
                             boost::optional<Matrix&> HN_G_B = boost::none,
                             boost::optional<Matrix&> H_B_S = boost::none) const {

            ///compute the estimated stereopoints
            gtsam::Rot3 R_G_B = TN_G_B.attitude();
            gtsam::Point3 P_G_B = TN_G_B.position();
            gtsam::Rot3 R_B_S = T_B_S.rotation();
            gtsam::Point3 P_B_S = T_B_S.translation();

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
            gtsam::Matrix3 dP_S_f_dV_G_B = Z_3x3;
            gtsam::Matrix39 dP_S_f_dTN_G_B;
            dP_S_f_dTN_G_B << dP_S_f_dR_G_B, dP_S_f_dP_G_B, dP_S_f_dV_G_B;
            gtsam::Matrix39 dimg_dTN_G_B;
            dimg_dTN_G_B = dimg_dP_S_f * dP_S_f_dTN_G_B;

            gtsam::Matrix3 dP_S_f_dR_B_S = skewSymmetric(P_S_f.vector());
            gtsam::Matrix3 dP_S_f_dP_B_S = -(R_B_S).transpose().matrix();
            gtsam::Matrix36 dP_S_f_dT_B_S;
            dP_S_f_dT_B_S << dP_S_f_dR_B_S, dP_S_f_dP_B_S;
            gtsam::Matrix36 dimg_dT_B_S;
            dimg_dT_B_S = dimg_dP_S_f * dP_S_f_dT_B_S;

            if (HN_G_B) *HN_G_B = dimg_dTN_G_B;
            if (H_B_S) *H_B_S = dimg_dT_B_S;

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
            ar & boost::serialization::make_nvp("NoiseModelFactor2",
                                                boost::serialization::base_object<Base>(*this));
            ar & BOOST_SERIALIZATION_NVP(measured_);
            ar & BOOST_SERIALIZATION_NVP(K_);
            ar & BOOST_SERIALIZATION_NVP(P_G_f_);
        }
    };

/// traits
//    template<class T1, class T2>
//    struct traits<StereoCalibMapFactor2<T1> > : public Testable<StereoCalibMapFactor2<T1> > {};




} // \ namespace gtsam




#endif //FACTORTEST_STEREOCALIBMAPFACTOR_H
