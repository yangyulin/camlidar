//
// Created by linde on 7/28/17.
//

#ifndef CATKIN_WS_TOTO_BETWEENCALIBFACTOR_H
#define CATKIN_WS_TOTO_BETWEENCALIBFACTOR_H

/* ----------------------------------------------------------------------------

 * -------------------------------------------------------------------------- */

/**
 *  @file   BetweenCalibFactor.h
 *  @author Yulin Yang
 *  @brief  Header file for BetweenCalibfactor
 *  @date   July 28, 2017
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>
#include <boost/optional.hpp>

namespace gtsam {


    class GTSAM_EXPORT BetweenCalibFactor: public NoiseModelFactor3<Pose3, Pose3, Pose3> {

    private:

    typedef NoiseModelFactor3<Pose3, Pose3, Pose3> Base;

    Pose3 Tm_S1_S2_; // relpose measurement
    //Point3 nT_; ///< Position measurement in cartesian coordinates

    public:

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<BetweenCalibFactor> shared_ptr;

    /// Typedef to this class
    typedef BetweenCalibFactor This;

    /** default constructor - only use for serialization */
    //GPSFactor(): nT_(0, 0, 0) {}
    BetweenCalibFactor();

    //virtual ~GPSFactor() {}
    virtual ~BetweenCalibFactor() {};

    /**
     * @brief Constructor from a relative pose measurement.
     * @param Pose_1inG, Pose_2inG, Pose_Calib (Sensor in the Body frame)that will be constrained
     * @param PoseM_2in1
     * @param model Gaussian noise model
     */
    //GPSFactor(Key key, const Point3& gpsIn, const SharedNoiseModel& model) :
    //Base(model, key), nT_(gpsIn) {
    //}
    BetweenCalibFactor( Key T_G_B1_Key,
                        Key T_G_B2_Key,
                        Key T_B_S_Key,
                        const Pose3 Tm_S1_S2,
                        const SharedNoiseModel& model)  :
                        Base(model, T_G_B1_Key, T_G_B2_Key, T_B_S_Key),
                        Tm_S1_S2_(Tm_S1_S2) {};

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /// print
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
    DefaultKeyFormatter) const;

    /// equals
    virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const;

    /// vector of errors
    Vector evaluateError(const Pose3& T_G_B1,
                         const Pose3& T_G_B2,
                         const Pose3& T_B_S,
                         boost::optional<Matrix&> H_G_B1 = boost::none,
                         boost::optional<Matrix&> H_G_B2 = boost::none,
                         boost::optional<Matrix&> H_B_S = boost::none) const;

    //inline const Point3 & measurementIn() const {
    //    return nT_;
    //}
    inline const Pose3 & measurementIn() const {
        return Tm_S1_S2_;
    }

    /*
    /**
     *  Convenience function to estimate state at time t, given two GPS
     *  readings (in local NED Cartesian frame) bracketing t
     *  Assumes roll is zero, calculates yaw and pitch from NED1->NED2 vector.

    static std::pair<Pose3, Vector3> EstimateState(double t1, const Point3& NED1,
                                                  double t2, const Point3& NED2, double timestamp);
    */
    private:

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar
        & boost::serialization::make_nvp("NoiseModelFactor3",
                                         boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(Tm_S1_S2_);
    }
};



/**
 * Version of BetweenCalibFactor for NavState
 * @addtogroup Navigation
 */
class GTSAM_EXPORT BetweenCalibFactor2: public NoiseModelFactor3<NavState, NavState, Pose3> {

private:

typedef NoiseModelFactor3<NavState, NavState, Pose3> Base;

//Point3 nT_; ///< Position measurement in cartesian coordinates
Pose3 Tm_S1_S2_;

public:

/// shorthand for a smart pointer to a factor
typedef boost::shared_ptr<BetweenCalibFactor2> shared_ptr;

/// Typedef to this class
typedef BetweenCalibFactor2 This;

/// default constructor - only use for serialization
//GPSFactor2():nT_(0, 0, 0) {}
BetweenCalibFactor2();

//virtual ~GPSFactor2() {}
virtual ~BetweenCalibFactor2() {}

/// Constructor from a measurement in a Cartesian frame.
//GPSFactor2(Key key, const Point3& gpsIn, const SharedNoiseModel& model) :
//Base(model, key), nT_(gpsIn) {
//};
BetweenCalibFactor2(Key TN_G_B1_Key,
                    Key TN_G_B2_Key,
                    Key T_B_S_Key,
                    const Pose3 Tm_S1_S2,
                    const SharedNoiseModel& model) :
                    Base(model, TN_G_B1_Key, TN_G_B2_Key, T_B_S_Key),
                    Tm_S1_S2_(Tm_S1_S2) {};

/// @return a deep copy of this factor
virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

/// print
virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
DefaultKeyFormatter) const;

/// equals
virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const;

/// vector of errors
Vector evaluateError(const NavState& TN_G_B1_Key,
                     const NavState& TN_G_B2_Key,
                     const Pose3& T_B_S_Key,
                     boost::optional<Matrix&> H_G_B1 = boost::none,
                     boost::optional<Matrix&> H_G_B2 = boost::none,
                     boost::optional<Matrix&> H_B_S = boost::none) const;

//inline const Point3 & measurementIn() const {
//    return nT_;
//}
inline const Pose3 & measurementIn() const {
    return Tm_S1_S2_;
}

private:

/// Serialization function
friend class boost::serialization::access;
template<class ARCHIVE>
void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
    & boost::serialization::make_nvp("NoiseModelFactor3",
                                     boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(Tm_S1_S2_);
}
};

} /// namespace gtsam



#endif //CATKIN_WS_TOTO_BETWEENCALIBFACTOR_H
