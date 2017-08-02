//
// Created by linde on 7/28/17.
//

/**
 *  @file   BetweenCalibFactor.cpp
 *  @author Yulin Yang
 *  @brief  Implementation file for BetweenCalib factor
 *  @date   July 29, 2017
 **/

#include "BetweenCalibFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
    void BetweenCalibFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
        std::cout << s << "BetweenCalibFactor, z= "<<endl;
        traits<Pose3>::Print(Tm_S1_S2_);
        Base::print("", keyFormatter);
        //cout << s << "GPSFactor on " << keyFormatter(key()) << "\n";
        //cout << "  GPS measurement: " << nT_ << "\n";
        //noiseModel_->print("  noise model: ");
    }

//***************************************************************************
    bool BetweenCalibFactor::equals(const NonlinearFactor& expected, double tol) const {
        const This* e = dynamic_cast<const This*>(&expected);
        return e
                && Base::equals(expected, tol)
                && traits<Pose3>::Equals(Tm_S1_S2_, e->Tm_S1_S2_, tol);
        //return e != NULL && Base::equals(*e, tol) && traits<Point3>::Equals(nT_, e->nT_, tol);
    }

//***************************************************************************
//    bool GPSFactor::equals(const NonlinearFactor& expected, double tol) const {
//        const This* e = dynamic_cast<const This*>(&expected);
//        return e != NULL && Base::equals(*e, tol) && traits<Point3>::Equals(nT_, e->nT_, tol);
//    }


    //***************************************************************************
    Vector BetweenCalibFactor::evaluateError(const Pose3& T_G_B1,
                                             const Pose3& T_G_B2,
                                             const Pose3& T_B_S,
                                             boost::optional<Matrix&> H_G_B1,
                                             boost::optional<Matrix&> H_G_B2,
                                             boost::optional<Matrix&> H_B_S) const {

        gtsam::Pose3 T_B1_S2 = T_G_B1.inverse() * T_G_B2 * T_B_S;
        gtsam::Pose3 T_hat_S1_S2 = T_B_S.inverse() * T_B1_S2;
        gtsam::Pose3 z_tilde = Tm_S1_S2_.inverse() * T_hat_S1_S2;
        gtsam::Vector6 error = Pose3::Logmap(z_tilde);

        gtsam::Matrix3 dzRdR_G_B1 = - T_B1_S2.rotation().transpose().matrix();
        gtsam::Matrix3 dzRdP_G_B1 = Z_3x3;
        gtsam::Matrix3 dzPdR_G_B1 = T_B1_S2.rotation().transpose().matrix() * skewSymmetric(T_B1_S2.translation().vector());
        gtsam::Matrix3 dzPdP_G_B1 = dzRdR_G_B1;
        gtsam::Matrix6 JH_G_B1;
        gtsam::insertSub(JH_G_B1, dzRdR_G_B1, 0, 0);
        gtsam::insertSub(JH_G_B1, dzRdP_G_B1, 0, 3);
        gtsam::insertSub(JH_G_B1, dzPdR_G_B1, 3, 0);
        gtsam::insertSub(JH_G_B1, dzPdP_G_B1, 3, 3);
        //JH_G_B1 << dzRdR_G_B1, dzRdP_G_B1, dzPdR_G_B1, dzPdP_G_B1;

        gtsam::Matrix3 dzRdR_G_B2 = T_B_S.rotation().transpose().matrix();
        gtsam::Matrix3 dzRdP_G_B2 = Z_3x3;
        gtsam::Matrix3 dzPdR_G_B2 = -T_B_S.rotation().transpose().matrix() * skewSymmetric(T_B_S.translation().vector());
        gtsam::Matrix3 dzPdP_G_B2 = dzRdR_G_B2;
        gtsam::Matrix6 JH_G_B2;
        gtsam::insertSub(JH_G_B2, dzRdR_G_B2, 0, 0);
        gtsam::insertSub(JH_G_B2, dzRdP_G_B2, 0, 3);
        gtsam::insertSub(JH_G_B2, dzPdR_G_B2, 3, 0);
        gtsam::insertSub(JH_G_B2, dzPdP_G_B2, 3, 3);
        //JH_G_B2 << dzRdR_G_B2, dzRdP_G_B2, dzPdR_G_B2, dzPdP_G_B2;

        gtsam::Matrix3 dzRdR_B_S = I_3x3 - T_hat_S1_S2.rotation().transpose().matrix();
        gtsam::Matrix3 dzRdP_B_S = Z_3x3;
        gtsam::Matrix3 dzPdR_B_S = T_hat_S1_S2.rotation().transpose().matrix() * skewSymmetric(T_hat_S1_S2.translation().vector());
        gtsam::Matrix3 dzPdP_B_S = dzRdR_B_S;
        gtsam::Matrix6 JH_B_S;
        gtsam::insertSub(JH_B_S, dzRdR_B_S, 0, 0);
        gtsam::insertSub(JH_B_S, dzRdP_B_S, 0, 3);
        gtsam::insertSub(JH_B_S, dzPdR_B_S, 3, 0);
        gtsam::insertSub(JH_B_S, dzPdP_B_S, 3, 3);
        //JH_B_S << dzRdR_B_S, dzRdP_B_S, dzPdR_B_S, dzPdP_B_S;

        if (H_G_B1) *H_G_B1 = JH_G_B1;
        if (H_G_B2) *H_G_B2 = JH_G_B2;
        if (H_B_S) *H_B_S = JH_B_S;
        return error;
    }

//***************************************************************************

//***************************************************************************
    void BetweenCalibFactor2::print(const string& s, const KeyFormatter& keyFormatter) const {
        std::cout << s << "BetweenCalibFactor2, z= "<<endl;
        traits<Pose3>::Print(Tm_S1_S2_);
        Base::print("", keyFormatter);
        //cout << s << "GPSFactor2 on " << keyFormatter(key()) << "\n";
        //cout << "  GPS measurement: " << nT_.transpose() << endl;
        //noiseModel_->print("  noise model: ");
    }

//***************************************************************************
    bool BetweenCalibFactor2::equals(const NonlinearFactor& expected, double tol) const {
        const This* e = dynamic_cast<const This*>(&expected);
        return e
               && Base::equals(expected, tol)
               && traits<Pose3>::Equals(Tm_S1_S2_, e->Tm_S1_S2_, tol);

        //const This* e = dynamic_cast<const This*>(&expected);
        //return e != NULL && Base::equals(*e, tol) &&
        //       traits<Point3>::Equals(nT_, e->nT_, tol);
    }

//***************************************************************************
    Vector BetweenCalibFactor2::evaluateError(const NavState& TN_G_B1,
                                              const NavState& TN_G_B2,
                                              const Pose3& T_B_S,
                                              boost::optional<Matrix&> HN_G_B1,
                                              boost::optional<Matrix&> HN_G_B2,
                                              boost::optional<Matrix&> H_B_S) const {

        gtsam::Pose3 T_G_B1 = TN_G_B1.pose();
        gtsam::Pose3 T_G_B2 = TN_G_B2.pose();
        gtsam::Pose3 T_B1_S2 = T_G_B1.inverse() * T_G_B2 * T_B_S;
        gtsam::Pose3 T_hat_S1_S2 = T_B_S.inverse() * T_B1_S2;
        gtsam::Pose3 z_tilde = Tm_S1_S2_.inverse() * T_hat_S1_S2;
        gtsam::Vector6 error = Pose3::Logmap(z_tilde);

        gtsam::Matrix3 dzRdR_G_B1 = - T_B1_S2.rotation().transpose().matrix();
        gtsam::Matrix3 dzRdP_G_B1 = Z_3x3;
        gtsam::Matrix3 dzPdR_G_B1 = T_B1_S2.rotation().transpose().matrix() * skewSymmetric(T_B1_S2.translation().vector());
        gtsam::Matrix3 dzPdP_G_B1 = dzRdR_G_B1;
        gtsam::Matrix3 dzRdV_G_B1 = Z_3x3;
        gtsam::Matrix3 dzPdV_G_B1 = Z_3x3;
        gtsam::Matrix69 JH_G_B1;
        //JH_G_B1.setZero();
        gtsam::insertSub(JH_G_B1, dzRdR_G_B1, 0, 0);
        gtsam::insertSub(JH_G_B1, dzRdP_G_B1, 0, 3);
        gtsam::insertSub(JH_G_B1, dzRdV_G_B1, 0, 6);
        gtsam::insertSub(JH_G_B1, dzPdR_G_B1, 3, 0);
        gtsam::insertSub(JH_G_B1, dzPdP_G_B1, 3, 3);
        gtsam::insertSub(JH_G_B1, dzPdV_G_B1, 3, 6);
        //JH_G_B1 << dzRdR_G_B1, dzRdP_G_B1, dzPdR_G_B1, dzPdP_G_B1;

        gtsam::Matrix3 dzRdR_G_B2 = T_B_S.rotation().transpose().matrix();
        gtsam::Matrix3 dzRdP_G_B2 = Z_3x3;
        gtsam::Matrix3 dzPdR_G_B2 = -T_B_S.rotation().transpose().matrix() * skewSymmetric(T_B_S.translation().vector());
        gtsam::Matrix3 dzPdP_G_B2 = dzRdR_G_B2;
        gtsam::Matrix3 dzRdV_G_B2 = Z_3x3;
        gtsam::Matrix3 dzPdV_G_B2 = Z_3x3;
        gtsam::Matrix69 JH_G_B2;
        gtsam::insertSub(JH_G_B2, dzRdR_G_B2, 0, 0);
        gtsam::insertSub(JH_G_B2, dzRdP_G_B2, 0, 3);
        gtsam::insertSub(JH_G_B2, dzRdV_G_B2, 0, 6);
        gtsam::insertSub(JH_G_B2, dzPdR_G_B2, 3, 0);
        gtsam::insertSub(JH_G_B2, dzPdP_G_B2, 3, 3);
        gtsam::insertSub(JH_G_B2, dzPdV_G_B2, 3, 6);
        //JH_G_B2 << dzRdR_G_B2, dzRdP_G_B2, dzPdR_G_B2, dzPdP_G_B2;

        gtsam::Matrix3 dzRdR_B_S = I_3x3 - T_hat_S1_S2.rotation().transpose().matrix();
        gtsam::Matrix3 dzRdP_B_S = Z_3x3;
        gtsam::Matrix3 dzPdR_B_S = T_hat_S1_S2.rotation().transpose().matrix() * skewSymmetric(T_hat_S1_S2.translation().vector());
        gtsam::Matrix3 dzPdP_B_S = dzRdR_B_S;
        gtsam::Matrix6 JH_B_S;
        gtsam::insertSub(JH_B_S, dzRdR_B_S, 0, 0);
        gtsam::insertSub(JH_B_S, dzRdP_B_S, 0, 3);
        gtsam::insertSub(JH_B_S, dzPdR_B_S, 3, 0);
        gtsam::insertSub(JH_B_S, dzPdP_B_S, 3, 3);
        //JH_B_S << dzRdR_B_S, dzRdP_B_S, dzPdR_B_S, dzPdP_B_S, dzVdR_B_S, dzVdP_B_S;


        if (HN_G_B1) *HN_G_B1 = JH_G_B1;
        if (HN_G_B2) *HN_G_B2 = JH_G_B2;
        if (H_B_S) *H_B_S = JH_B_S;

        return error;
    }

//***************************************************************************

}/// namespace gtsam
