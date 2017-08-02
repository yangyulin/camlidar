//
// Created by linde on 8/1/17.
//

#ifndef FACTORTEST_PLANECALIBFACTOR_H
#define FACTORTEST_PLANECALIBFACTOR_H

/**
 * @file    PlaneCalibMapFactor.h
 * @brief   A non-linear factor for Plane measurements
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
 * A plane constaint factor
 * @addtogroup SLAM
 */
    class GTSAM_EXPORT PlaneCalibMapFactor: public NoiseModelFactor2<Pose3, Pose3> {
    private:

        // Keep a copy of measurement and calibration for I/O
        Vector3 n_in_G_;                              ///< the plane normal direction
        double d_in_G_;                               /// the scalar of the plane n_in_G ^T * P_G_f + d_in_G = 0
        Point3 Pm_S_f_;                               ///< the feature point in the sensor frame

        // verbosity handling for Cheirality Exceptions
        //bool throwCheirality_;                       ///< If true, rethrows Cheirality exceptions (default: false)
        //bool verboseCheirality_;                     ///< If true, prints text for Cheirality exceptions (default: false)

    public:

        // shorthand for base class type
        typedef NoiseModelFactor2<Pose3, Pose3> Base;              ///< typedef for base class
        typedef PlaneCalibMapFactor This;                ///< typedef for this class (with templates)
        typedef boost::shared_ptr<PlaneCalibMapFactor> shared_ptr;  ///< typedef for shared pointer to this object
        //typedef POSE CamPose;                                       ///< typedef for Pose Lie Value type

        /**
         * Default constructor
         */
        PlaneCalibMapFactor() : n_in_G_(0,0,1), d_in_G_(0) {}

        /**
         * Constructor
         * @param measured is the Stereo Point measurement (u_l, u_r, v). v will be identical for left & right for rectified stereo pair
         * @param model is the noise model in on the measurement
         * @param poseKey the pose variable key
         * @param K the constant calibration
         * @param P_G_f is the feature point in the global frame
         */
        PlaneCalibMapFactor(const Point3& Pm_S_f,
                             const SharedNoiseModel& model,
                             Key T_G_B_Key,
                             Key T_B_S_Key,
                             Vector3 n_in_G,
                             double d_in_G) :
                Base(model, T_G_B_Key, T_B_S_Key), Pm_S_f_(Pm_S_f), n_in_G_(n_in_G), d_in_G_(d_in_G) {}


        /** Virtual destructor */
        virtual ~PlaneCalibMapFactor() {}

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
            Pm_S_f_.print(s + ".z");
            std::cout<<" The plane normal vector is " <<n_in_G_<<std::endl;
            std::cout<<" The plane scalar is " <<d_in_G_<<std::endl;
        }

        /**
         * equals
         */
        virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
            const PlaneCalibMapFactor * e = dynamic_cast<const PlaneCalibMapFactor *> (&f);
            return e
                   && Base::equals(f)
                   && Pm_S_f_.equals(e->Pm_S_f_, tol)
                   && fabs(n_in_G_(1) - e->n_in_G_(1)) < tol
                   && fabs(n_in_G_(2) - e->n_in_G_(2)) < tol
                   && fabs(n_in_G_(3) - e->n_in_G_(3)) < tol
                   && fabs(d_in_G_ - e->d_in_G_) < tol ;
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

            gtsam::Point3 P_B_f = R_B_S * Pm_S_f_ + P_B_S;
            gtsam::Point3 P_G_f = R_G_B * P_B_f + P_G_B;

            // The plane constraint
            gtsam::Vector1 error(n_in_G_.transpose() * P_G_f.vector() + d_in_G_);

            // compute the Jacobians
            gtsam::Matrix3 dP_G_f_dR_G_B = - R_G_B.matrix() * skewSymmetric(P_B_f.vector());
            gtsam::Matrix3 dP_G_f_dP_G_B = I_3x3;

            gtsam::Matrix3 dP_G_f_dR_B_S = - R_G_B.matrix() * R_B_S.matrix() * skewSymmetric(Pm_S_f_.vector());
            gtsam::Matrix3 dP_G_f_dP_B_S = R_G_B.matrix();

            gtsam::Matrix36 dP_G_f_dT_G_B;
            dP_G_f_dT_G_B << dP_G_f_dR_G_B, dP_G_f_dP_G_B;
            gtsam::Matrix36 dP_G_f_dT_B_S;
            dP_G_f_dT_B_S << dP_G_f_dR_B_S, dP_G_f_dP_B_S;



            if (H_G_B) *H_G_B = n_in_G_.transpose() * dP_G_f_dT_G_B;
            if (H_B_S) *H_B_S = n_in_G_.transpose() * dP_G_f_dT_B_S;

            return error;

        }

        /** return the measured */
        const Point3& measured() const {
            return Pm_S_f_;
        }

        /** return the plane info */
        inline const Vector3 planeNormal() const {
            return n_in_G_;
        }

        inline const double planeScalar() const {
            return d_in_G_;
        }


    private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive & ar, const unsigned int /*version*/) {
            ar & boost::serialization::make_nvp("NoiseModelFactor2",
                                                boost::serialization::base_object<Base>(*this));
            ar & BOOST_SERIALIZATION_NVP(Pm_S_f_);
            ar & BOOST_SERIALIZATION_NVP(n_in_G_);
            ar & BOOST_SERIALIZATION_NVP(d_in_G_);
        }
    };

/// traits
//    template<class T1, class T2>
//    struct traits<PlaneCalibMapFactor<T1> > : public Testable<PlaneCalibMapFactor<T1> > {};




/**
 * A Generic Plane Factor
 * @addtogroup SLAM
 */
    class GTSAM_EXPORT PlaneCalibMapFactor2: public NoiseModelFactor2<NavState, Pose3> {
    private:

        // Keep a copy of measurement and calibration for I/O
        Vector3 n_in_G_;                              ///< the plane normal direction
        double d_in_G_;                               /// the scalar of the plane n_in_G ^T * P_G_f + d_in_G = 0
        Point3 Pm_S_f_;                               ///< the feature point in the sensor frame

        // verbosity handling for Cheirality Exceptions
        //bool throwCheirality_;                       ///< If true, rethrows Cheirality exceptions (default: false)
        //bool verboseCheirality_;                     ///< If true, prints text for Cheirality exceptions (default: false)

    public:

        // shorthand for base class type
        typedef NoiseModelFactor2<NavState, Pose3> Base;              ///< typedef for base class
        typedef PlaneCalibMapFactor2 This;                ///< typedef for this class (with templates)
        typedef boost::shared_ptr<PlaneCalibMapFactor2> shared_ptr;  ///< typedef for shared pointer to this object
        //typedef POSE CamPose;                                       ///< typedef for Pose Lie Value type

        /**
         * Default constructor
         */
        PlaneCalibMapFactor2() : n_in_G_(0,0,1), d_in_G_(0) {}

        /**
         * Constructor
         * @param measured is the Stereo Point measurement (u_l, u_r, v). v will be identical for left & right for rectified stereo pair
         * @param model is the noise model in on the measurement
         * @param poseKey the pose variable key
         * @param K the constant calibration
         * @param P_G_f is the feature point in the global frame
         */
        PlaneCalibMapFactor2(const Point3& Pm_S_f,
                            const SharedNoiseModel& model,
                            Key TN_G_B_Key,
                            Key T_B_S_Key,
                            Vector3 n_in_G,
                            double d_in_G) :
                Base(model, TN_G_B_Key, T_B_S_Key), Pm_S_f_(Pm_S_f), n_in_G_(n_in_G), d_in_G_(d_in_G) {}


        /** Virtual destructor */
        virtual ~PlaneCalibMapFactor2() {}

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
            Pm_S_f_.print(s + ".z");
            std::cout<<" The plane normal vector is " <<n_in_G_<<std::endl;
            std::cout<<" The plane scalar is " <<d_in_G_<<std::endl;
        }

        /**
         * equals
         */
        virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
            const PlaneCalibMapFactor2 * e = dynamic_cast<const PlaneCalibMapFactor2 *> (&f);
            return e
                   && Base::equals(f)
                   && Pm_S_f_.equals(e->Pm_S_f_, tol)
                   && fabs(n_in_G_(1) - e->n_in_G_(1)) < tol
                   && fabs(n_in_G_(2) - e->n_in_G_(2)) < tol
                   && fabs(n_in_G_(3) - e->n_in_G_(3)) < tol
                   && fabs(d_in_G_ - e->d_in_G_) < tol ;
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

            gtsam::Point3 P_B_f = R_B_S * Pm_S_f_ + P_B_S;
            gtsam::Point3 P_G_f = R_G_B * P_B_f + P_G_B;

            // The plane constraint
            gtsam::Vector1 error(n_in_G_.transpose() * P_G_f.vector() + d_in_G_);

            // compute the Jacobians
            gtsam::Matrix3 dP_G_f_dR_G_B = - R_G_B.matrix() * skewSymmetric(P_B_f.vector());
            gtsam::Matrix3 dP_G_f_dP_G_B = I_3x3;
            gtsam::Matrix3 dP_G_f_dV_G_B = Z_3x3;

            gtsam::Matrix3 dP_G_f_dR_B_S = - R_G_B.matrix() * R_B_S.matrix() * skewSymmetric(Pm_S_f_.vector());
            gtsam::Matrix3 dP_G_f_dP_B_S = R_G_B.matrix();

            gtsam::Matrix39 dP_G_f_dTN_G_B;
            dP_G_f_dTN_G_B << dP_G_f_dR_G_B, dP_G_f_dP_G_B, dP_G_f_dV_G_B;
            gtsam::Matrix36 dP_G_f_dT_B_S;
            dP_G_f_dT_B_S << dP_G_f_dR_B_S, dP_G_f_dP_B_S;



            if (HN_G_B) *HN_G_B = n_in_G_.transpose() * dP_G_f_dTN_G_B;
            if (H_B_S) *H_B_S = n_in_G_.transpose() * dP_G_f_dT_B_S;

            return error;

        }

        /** return the measured */
        const Point3& measured() const {
            return Pm_S_f_;
        }

        /** return the plane info */
        inline const Vector3 planeNormal() const {
            return n_in_G_;
        }

        inline const double planeScalar() const {
            return d_in_G_;
        }


    private:
        /** Serialization function */
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive & ar, const unsigned int /*version*/) {
            ar & boost::serialization::make_nvp("NoiseModelFactor2",
                                                boost::serialization::base_object<Base>(*this));
            ar & BOOST_SERIALIZATION_NVP(Pm_S_f_);
            ar & BOOST_SERIALIZATION_NVP(n_in_G_);
            ar & BOOST_SERIALIZATION_NVP(d_in_G_);
        }
    };

/// traits
 //   template<class T1, class T2>
//    struct traits<PlaneCalibMapFactor2<T1> > : public Testable<PlaneCalibMapFactor2<T1> > {};







} // \ namespace gtsam



#endif //FACTORTEST_PLANECALIBFACTOR_H
