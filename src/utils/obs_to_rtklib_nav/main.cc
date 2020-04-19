/*!
 * \file main.cc
 * \brief rtklib_solver_test clone, accepts observables.dat
 * \author Mariusz Krej
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "GPS_L1_CA.h"

#include "geofunctions.h"
#include "gnss_sdr_supl_client.h"
#include "in_memory_configuration.h"
#include "rtklib_solver.h"
#include "rtklib_rtkpos.h"

//#include "tracking_true_obs_reader.h"
#include "observables_dump_reader.h"

#include "gpx_printer.h"

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <iomanip>
#include <iostream>
#include <string>


//#include <armadillo>
//#include <gtest/gtest.h>


#define LOG(severity) std::cout


//TODO Single vs PPP
#define USE_PPP (1)


rtk_t configure_rtklib_options()
{
    std::shared_ptr<InMemoryConfiguration> configuration;
    configuration = std::make_shared<InMemoryConfiguration>();
    std::string role = "rtklib_solver";

    /*
    positioning_mode=PPP_Static  ; options: Single, Static, Kinematic, PPP_Static, PPP_Kinematic
    iono_model=Broadcast ; options: OFF, Broadcast, SBAS, Iono-Free-LC, Estimate_STEC, IONEX
    PVT.trop_model=Saastamoinen ; options: OFF, Saastamoinen, SBAS, Estimate_ZTD, Estimate_ZTD_Grad
    */

    if (!USE_PPP)
    {
        // custom options
        configuration->set_property("rtklib_solver.positioning_mode", "Single");
        configuration->set_property("rtklib_solver.elevation_mask", "16");
        configuration->set_property("rtklib_solver.iono_model", "OFF");
        configuration->set_property("rtklib_solver.trop_model", "OFF");
        //configuration->set_property("rtklib_solver.iono_model", "Broadcast");
        //configuration->set_property("rtklib_solver.trop_model", "Saastamoinen");

    }
    else
    {
        configuration->set_property("rtklib_solver.positioning_mode", "PPP_Static");

        configuration->set_property("rtklib_solver.elevation_mask", "0");
        //configuration->set_property("rtklib_solver.elevation_mask", "6");
        //configuration->set_property("rtklib_solver.elevation_mask", "16");

        configuration->set_property("rtklib_solver.iono_model", "OFF");
        configuration->set_property("rtklib_solver.trop_model", "OFF");
        //configuration->set_property("rtklib_solver.iono_model", "Broadcast");
        //configuration->set_property("rtklib_solver.trop_model", "Saastamoinen");
    }
    //RTKLIB PVT solver options

    // Settings 1
    int positioning_mode = -1;
    std::string default_pos_mode("Single");

    configuration->set_property(role + ".positioning_mode", default_pos_mode);

    std::string positioning_mode_str = configuration->property(role + ".positioning_mode", default_pos_mode); /* (PMODE_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    if (positioning_mode_str == "Single")
        {
            positioning_mode = PMODE_SINGLE;
        }
    if (positioning_mode_str == "Static")
        {
            positioning_mode = PMODE_STATIC;
        }
    if (positioning_mode_str == "Kinematic")
        {
            positioning_mode = PMODE_KINEMA;
        }
    if (positioning_mode_str == "PPP_Static")
        {
            positioning_mode = PMODE_PPP_STATIC;
        }
    if (positioning_mode_str == "PPP_Kinematic")
        {
            positioning_mode = PMODE_PPP_KINEMA;
        }

    //positioning_mode = PMODE_PPP_STATIC;


    if (positioning_mode == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of positioning mode." << std::endl;
            std::cout << "positioning_mode possible values: Single / Static / Kinematic / PPP_Static / PPP_Kinematic" << std::endl;
            std::cout << "positioning_mode specified value: " << positioning_mode_str << std::endl;
            std::cout << "Setting positioning_mode to Single" << std::endl;
            positioning_mode = PMODE_SINGLE;
        }

    int num_bands = 1;

    //    if ((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) num_bands = 1;
    //    if (((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) && ((gps_2S_count > 0) || (glo_2G_count > 0))) num_bands = 2;
    //    if (((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) && ((gal_E5a_count > 0) || (gal_E5b_count > 0) || (gps_L5_count > 0))) num_bands = 2;
    //    if (((gps_1C_count > 0) || (gal_1B_count > 0) || (glo_1G_count > 0)) && ((gps_2S_count > 0) || (glo_2G_count > 0)) && ((gal_E5a_count > 0) || (gal_E5b_count > 0) || (gps_L5_count > 0))) num_bands = 3;

    int number_of_frequencies = configuration->property(role + ".num_bands", num_bands); /* (1:L1, 2:L1+L2, 3:L1+L2+L5) */
    if ((number_of_frequencies < 1) || (number_of_frequencies > 3))
        {
            //warn user and set the default
            number_of_frequencies = num_bands;
        }

    double elevation_mask = configuration->property(role + ".elevation_mask", 15.0);
    if ((elevation_mask < 0.0) || (elevation_mask > 90.0))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Elevation Mask. Setting to default value of 15.0 degrees";
            elevation_mask = 15.0;
        }

    int dynamics_model = configuration->property(role + ".dynamics_model", 0); /*  dynamics model (0:none, 1:velocity, 2:accel) */
    if ((dynamics_model < 0) || (dynamics_model > 2))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Dynamics Model configuration. Setting to default value of (0:none)";
            dynamics_model = 0;
        }

    std::string default_iono_model("OFF");
    std::string iono_model_str = configuration->property(role + ".iono_model", default_iono_model); /*  (IONOOPT_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    int iono_model = -1;
    if (iono_model_str == "OFF")
        {
            iono_model = IONOOPT_OFF;
        }
    if (iono_model_str == "Broadcast")
        {
            iono_model = IONOOPT_BRDC;
        }
    if (iono_model_str == "SBAS")
        {
            iono_model = IONOOPT_SBAS;
        }
    if (iono_model_str == "Iono-Free-LC")
        {
            iono_model = IONOOPT_IFLC;
        }
    if (iono_model_str == "Estimate_STEC")
        {
            iono_model = IONOOPT_EST;
        }
    if (iono_model_str == "IONEX")
        {
            iono_model = IONOOPT_TEC;
        }
    if (iono_model == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of ionospheric model." << std::endl;
            std::cout << "iono_model possible values: OFF / Broadcast / SBAS / Iono-Free-LC / Estimate_STEC / IONEX" << std::endl;
            std::cout << "iono_model specified value: " << iono_model_str << std::endl;
            std::cout << "Setting iono_model to OFF" << std::endl;
            iono_model = IONOOPT_OFF; /* 0: ionosphere option: correction off */
        }

    std::string default_trop_model("OFF");
    int trop_model = -1;
    std::string trop_model_str = configuration->property(role + ".trop_model", default_trop_model); /*  (TROPOPT_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    if (trop_model_str == "OFF")
        {
            trop_model = TROPOPT_OFF;
        }
    if (trop_model_str == "Saastamoinen")
        {
            trop_model = TROPOPT_SAAS;
        }
    if (trop_model_str == "SBAS")
        {
            trop_model = TROPOPT_SBAS;
        }
    if (trop_model_str == "Estimate_ZTD")
        {
            trop_model = TROPOPT_EST;
        }
    if (trop_model_str == "Estimate_ZTD_Grad")
        {
            trop_model = TROPOPT_ESTG;
        }
    if (trop_model == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of tropospheric model." << std::endl;
            std::cout << "trop_model possible values: OFF / Saastamoinen / SBAS / Estimate_ZTD / Estimate_ZTD_Grad" << std::endl;
            std::cout << "trop_model specified value: " << trop_model_str << std::endl;
            std::cout << "Setting trop_model to OFF" << std::endl;
            trop_model = TROPOPT_OFF;
        }

    /* RTKLIB positioning options */
    int sat_PCV = 0; /*  Set whether the satellite antenna PCV (phase center variation) model is used or not. This feature requires a Satellite Antenna PCV File. */
    int rec_PCV = 0; /*  Set whether the receiver antenna PCV (phase center variation) model is used or not. This feature requires a Receiver Antenna PCV File. */

    /* Set whether the phase windup correction for PPP modes is applied or not. Only applicable to PPP‐* modes.*/
    int phwindup = configuration->property(role + ".phwindup", 0);

    /* Set whether the GPS Block IIA satellites in eclipse are excluded or not.
    The eclipsing Block IIA satellites often degrade the PPP solutions due to unpredicted behavior of yaw‐attitude. Only applicable to PPP‐* modes.*/
    int reject_GPS_IIA = configuration->property(role + ".reject_GPS_IIA", 0);

    /* Set whether RAIM (receiver autonomous integrity monitoring) FDE (fault detection and exclusion) feature is enabled or not.
    In case of RAIM FDE enabled, a satellite is excluded if SSE (sum of squared errors) of residuals is over a threshold.
    The excluded satellite is selected to indicate the minimum SSE. */
    int raim_fde = configuration->property(role + ".raim_fde", 0);

    int earth_tide = configuration->property(role + ".earth_tide", 0);

    int nsys = SYS_GPS;
    //    if ((gps_1C_count > 0) || (gps_2S_count > 0) || (gps_L5_count > 0)) nsys += SYS_GPS;
    //    if ((gal_1B_count > 0) || (gal_E5a_count > 0) || (gal_E5b_count > 0)) nsys += SYS_GAL;
    //    if ((glo_1G_count > 0) || (glo_2G_count > 0)) nsys += SYS_GLO;
    int navigation_system = configuration->property(role + ".navigation_system", nsys); /* (SYS_XXX) see src/algorithms/libs/rtklib/rtklib.h */
    if ((navigation_system < 1) || (navigation_system > 255))                           /* GPS: 1   SBAS: 2   GPS+SBAS: 3 Galileo: 8  Galileo+GPS: 9 GPS+SBAS+Galileo: 11 All: 255 */
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Navigation System. Setting to default value of (0:none)";
            navigation_system = nsys;
        }

    // Settings 2
    std::string default_gps_ar("Continuous");
    std::string integer_ambiguity_resolution_gps_str = configuration->property(role + ".AR_GPS", default_gps_ar); /* Integer Ambiguity Resolution mode for GPS (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
    int integer_ambiguity_resolution_gps = -1;
    if (integer_ambiguity_resolution_gps_str == "OFF")
        {
            integer_ambiguity_resolution_gps = ARMODE_OFF;
        }
    if (integer_ambiguity_resolution_gps_str == "Continuous")
        {
            integer_ambiguity_resolution_gps = ARMODE_CONT;
        }
    if (integer_ambiguity_resolution_gps_str == "Instantaneous")
        {
            integer_ambiguity_resolution_gps = ARMODE_INST;
        }
    if (integer_ambiguity_resolution_gps_str == "Fix-and-Hold")
        {
            integer_ambiguity_resolution_gps = ARMODE_FIXHOLD;
        }
    if (integer_ambiguity_resolution_gps_str == "PPP-AR")
        {
            integer_ambiguity_resolution_gps = ARMODE_PPPAR;
        }
    if (integer_ambiguity_resolution_gps == -1)
        {
            //warn user and set the default
            std::cout << "WARNING: Bad specification of GPS ambiguity resolution method." << std::endl;
            std::cout << "AR_GPS possible values: OFF / Continuous / Instantaneous / Fix-and-Hold / PPP-AR" << std::endl;
            std::cout << "AR_GPS specified value: " << integer_ambiguity_resolution_gps_str << std::endl;
            std::cout << "Setting AR_GPS to OFF" << std::endl;
            integer_ambiguity_resolution_gps = ARMODE_OFF;
        }

    int integer_ambiguity_resolution_glo = configuration->property(role + ".AR_GLO", 1); /* Integer Ambiguity Resolution mode for GLONASS (0:off,1:on,2:auto cal,3:ext cal) */
    if ((integer_ambiguity_resolution_glo < 0) || (integer_ambiguity_resolution_glo > 3))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Integer Ambiguity Resolution for GLONASS . Setting to default value of (1:on)";
            integer_ambiguity_resolution_glo = 1;
        }

    int integer_ambiguity_resolution_bds = configuration->property(role + ".AR_DBS", 1); /* Integer Ambiguity Resolution mode for BEIDOU (0:off,1:on) */
    if ((integer_ambiguity_resolution_bds < 0) || (integer_ambiguity_resolution_bds > 1))
        {
            //warn user and set the default
            LOG(WARNING) << "Erroneous Integer Ambiguity Resolution for BEIDOU . Setting to default value of (1:on)";
            integer_ambiguity_resolution_bds = 1;
        }

    double min_ratio_to_fix_ambiguity = configuration->property(role + ".min_ratio_to_fix_ambiguity", 3.0); /* Set the integer ambiguity validation threshold for ratio‐test,
                                                                                                               which uses the ratio of squared residuals of the best integer vector to the second‐best vector. */

    int min_lock_to_fix_ambiguity = configuration->property(role + ".min_lock_to_fix_ambiguity", 0); /* Set the minimum lock count to fix integer ambiguity.
                                                                                                         If the lock count is less than the value, the ambiguity is excluded from the fixed integer vector. */

    double min_elevation_to_fix_ambiguity = configuration->property(role + ".min_elevation_to_fix_ambiguity", 0.0); /* Set the minimum elevation (deg) to fix integer ambiguity.
                                                                                                                        If the elevation of the satellite is less than the value, the ambiguity is excluded from the fixed integer vector. */

    int outage_reset_ambiguity = configuration->property(role + ".outage_reset_ambiguity", 5); /* Set the outage count to reset ambiguity. If the data outage count is over the value, the estimated ambiguity is reset to the initial value.  */

    double slip_threshold = configuration->property(role + ".slip_threshold", 0.05); /* set the cycle‐slip threshold (m) of geometry‐free LC carrier‐phase difference between epochs */

    double threshold_reject_gdop = configuration->property(role + ".threshold_reject_gdop", 30.0); /* reject threshold of GDOP. If the GDOP is over the value, the observable is excluded for the estimation process as an outlier. */

    double threshold_reject_innovation = configuration->property(role + ".threshold_reject_innovation", 30.0); /* reject threshold of innovation (m). If the innovation is over the value, the observable is excluded for the estimation process as an outlier. */

    int number_filter_iter = configuration->property(role + ".number_filter_iter", 1); /* Set the number of iteration in the measurement update of the estimation filter.
                                                                                         If the baseline length is very short like 1 m, the iteration may be effective to handle
                                                                                         the nonlinearity of measurement equation. */

    /// Statistics
    double bias_0 = configuration->property(role + ".bias_0", 30.0);

    double iono_0 = configuration->property(role + ".iono_0", 0.03);

    double trop_0 = configuration->property(role + ".trop_0", 0.3);

    double sigma_bias = configuration->property(role + ".sigma_bias", 1e-4); /* Set the process noise standard deviation of carrier‐phase
                                                                                bias (ambiguity) (cycle/sqrt(s)) */

    double sigma_iono = configuration->property(role + ".sigma_iono", 1e-3); /* Set the process noise standard deviation of vertical ionospheric delay per 10 km baseline (m/sqrt(s)). */

    double sigma_trop = configuration->property(role + ".sigma_trop", 1e-4); /* Set the process noise standard deviation of zenith tropospheric delay (m/sqrt(s)). */

    double sigma_acch = configuration->property(role + ".sigma_acch", 1e-1); /* Set the process noise standard deviation of the receiver acceleration as
                                                                                the horizontal component. (m/s2/sqrt(s)). If Receiver Dynamics is set to OFF, they are not used. */

    double sigma_accv = configuration->property(role + ".sigma_accv", 1e-2); /* Set the process noise standard deviation of the receiver acceleration as
                                                                                the vertical component. (m/s2/sqrt(s)). If Receiver Dynamics is set to OFF, they are not used. */

    double sigma_pos = configuration->property(role + ".sigma_pos", 0.0);

    double code_phase_error_ratio_l1 = configuration->property(role + ".code_phase_error_ratio_l1", 100.0);
    double code_phase_error_ratio_l2 = configuration->property(role + ".code_phase_error_ratio_l2", 100.0);
    double code_phase_error_ratio_l5 = configuration->property(role + ".code_phase_error_ratio_l5", 100.0);
    double carrier_phase_error_factor_a = configuration->property(role + ".carrier_phase_error_factor_a", 0.003);
    double carrier_phase_error_factor_b = configuration->property(role + ".carrier_phase_error_factor_b", 0.003);

    snrmask_t snrmask = {{}, {{}, {}}};

    prcopt_t rtklib_configuration_options = {
        positioning_mode,                                                                  /* positioning mode (PMODE_XXX) see src/algorithms/libs/rtklib/rtklib.h */
        0,                                                                                 /* solution type (0:forward,1:backward,2:combined) */
        number_of_frequencies,                                                             /* number of frequencies (1:L1, 2:L1+L2, 3:L1+L2+L5)*/
        navigation_system,                                                                 /* navigation system  */
        elevation_mask * D2R,                                                              /* elevation mask angle (degrees) */
        snrmask,                                                                           /* snrmask_t snrmask    SNR mask */
        0,                                                                                 /* satellite ephemeris/clock (EPHOPT_XXX) */
        integer_ambiguity_resolution_gps,                                                  /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
        integer_ambiguity_resolution_glo,                                                  /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
        integer_ambiguity_resolution_bds,                                                  /* BeiDou AR mode (0:off,1:on) */
        outage_reset_ambiguity,                                                            /* obs outage count to reset bias */
        min_lock_to_fix_ambiguity,                                                         /* min lock count to fix ambiguity */
        10,                                                                                /* min fix count to hold ambiguity */
        1,                                                                                 /* max iteration to resolve ambiguity */
        iono_model,                                                                        /* ionosphere option (IONOOPT_XXX) */
        trop_model,                                                                        /* troposphere option (TROPOPT_XXX) */
        dynamics_model,                                                                    /* dynamics model (0:none, 1:velocity, 2:accel) */
        earth_tide,                                                                        /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
        number_filter_iter,                                                                /* number of filter iteration */
        0,                                                                                 /* code smoothing window size (0:none) */
        0,                                                                                 /* interpolate reference obs (for post mission) */
        0,                                                                                 /* sbssat_t sbssat  SBAS correction options */
        0,                                                                                 /* sbsion_t sbsion[MAXBAND+1] SBAS satellite selection (0:all) */
        0,                                                                                 /* rover position for fixed mode */
        0,                                                                                 /* base position for relative mode */
                                                                                           /*    0:pos in prcopt,  1:average of single pos, */
                                                                                           /*    2:read from file, 3:rinex header, 4:rtcm pos */
        {code_phase_error_ratio_l1, code_phase_error_ratio_l2, code_phase_error_ratio_l5}, /* eratio[NFREQ] code/phase error ratio */
        {100.0, carrier_phase_error_factor_a, carrier_phase_error_factor_b, 0.0, 1.0},     /* err[5]:  measurement error factor [0]:reserved, [1-3]:error factor a/b/c of phase (m) , [4]:doppler frequency (hz) */
        {bias_0, iono_0, trop_0},                                                          /* std[3]: initial-state std [0]bias,[1]iono [2]trop*/
        {sigma_bias, sigma_iono, sigma_trop, sigma_acch, sigma_accv, sigma_pos},           /* prn[6] process-noise std */
        5e-12,                                                                             /* sclkstab: satellite clock stability (sec/sec) */
        {min_ratio_to_fix_ambiguity, 0.9999, 0.25, 0.1, 0.05, 0.0, 0.0, 0.0},              /* thresar[8]: AR validation threshold */
        min_elevation_to_fix_ambiguity,                                                    /* elevation mask of AR for rising satellite (deg) */
        0.0,                                                                               /* elevation mask to hold ambiguity (deg) */
        slip_threshold,                                                                    /* slip threshold of geometry-free phase (m) */
        30.0,                                                                              /* max difference of time (sec) */
        threshold_reject_innovation,                                                       /* reject threshold of innovation (m) */
        threshold_reject_gdop,                                                             /* reject threshold of gdop */
        {},                                                                                /* double baseline[2] baseline length constraint {const,sigma} (m) */
        {},                                                                                /* double ru[3]  rover position for fixed mode {x,y,z} (ecef) (m) */
        {},                                                                                /* double rb[3]  base position for relative mode {x,y,z} (ecef) (m) */
        {"", ""},                                                                          /* char anttype[2][MAXANT]  antenna types {rover,base}  */
        {{}, {}},                                                                          /* double antdel[2][3]   antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
        {},                                                                                /* pcv_t pcvr[2]   receiver antenna parameters {rov,base} */
        {},                                                                                /* unsigned char exsats[MAXSAT]  excluded satellites (1:excluded, 2:included) */
        0,                                                                                 /* max averaging epoches */
        0,                                                                                 /* initialize by restart */
        1,                                                                                 /* output single by dgps/float/fix/ppp outage */
        {"", ""},                                                                          /* char rnxopt[2][256]   rinex options {rover,base} */
        {sat_PCV, rec_PCV, phwindup, reject_GPS_IIA, raim_fde},                            /* posopt[6] positioning options [0]: satellite and receiver antenna PCV model; [1]: interpolate antenna parameters; [2]: apply phase wind-up correction for PPP modes; [3]: exclude measurements of GPS Block IIA satellites satellite [4]: RAIM FDE (fault detection and exclusion) [5]: handle day-boundary clock jump */
        0,                                                                                 /* solution sync mode (0:off,1:on) */
        {{}, {}},                                                                          /* odisp[2][6*11] ocean tide loading parameters {rov,base} */
        {{}, {{}, {}}, {{}, {}}, {}, {}},                                                  /* exterr_t exterr   extended receiver error model */
        0,                                                                                 /* disable L2-AR */
        {}                                                                                 /* char pppopt[256]   ppp option   "-GAP_RESION="  default gap to reset iono parameters (ep) */
    };

    rtk_t rtk;
    rtkinit(&rtk, &rtklib_configuration_options);
    return rtk;
}



int main() //int argc, char** argv)
{
    // TODO file sel

    //std::string true_obs_file = std::string("/home/mk/Gnss/gnss-sdr-my/observables.dat");

    // opcja filtrowania efemeryd wg TOE, -1 oznacza brak filtrowania
    int find_toe = -1;

#if 0
    const int error_bound = 200;

    // gps-sdr-sim ref pos
    arma::vec true_r_eb_e = arma::vec({ 3655463.659, 1404112.314, 5017924.853 });

    int dump_n_channels = 5;
    std::string true_obs_file = std::string("/home/mk/Gnss/Results/2020-04-05/1/observables.dat");
    //std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-05/1/gps_ephemeris.xml";
    std::string eph_xml_filename = "/home/mk/Gnss/gnss-sdr-my/gps_ephemeris_full_brdc3540_14n.xml";
    find_toe = 518400;

#endif


#if 0 && !USE_PPP

    // zapis z FPGA 16MHz bez pomiarów fazowych
    // Single 3D SD[m] = 16.93
    const int error_bound = 200;
    //arma::vec true_r_eb_e = { 3655465.449, 1404114.512, 5017927.510};
    arma::vec true_r_eb_e = { 3655463.659, 1404112.314, 5017924.853 };

    // dla błędów < 20
    // Single 3D SD[m] = 4.33
    //arma::vec true_r_eb_e = { 3655465.449, 1404112.665, 5017926.698};

    int dump_n_channels = 5;
    std::string true_obs_file = "/home/mk/Gnss/Results/2019-09-07/2/ticksExSavIQ.bin_obs.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-05/1/gps_ephemeris.xml";

#endif

#if 0
    // Single, iono+tropo OFF, mean - ref [m] = 7.04, 3D SD[m] = 18.36 ???
    // arma::vec true_r_eb_e = { 3655479.590, 1404123.428, 5017946.443};
    // 3D SD[m] = 92m

    int dump_n_channels = 8;
    std::string true_obs_file = "/home/mk/Gnss/Results/2020-04-07/1/OnlineScanNav-gnss-sim-observables-ok.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-07/1/gps_ephemeris.xml";
#endif

#if 0
    // PPP_Static, elev 0, iono off, sd-3d[m] = 0.0005-NIEPRAWDA
    // ref = { 3655482.807, 1404118.578, 5017917.000};
    int dump_n_channels = 8;
    std::string true_obs_file = "/home/mk/Gnss/Results/2020-04-07/2-OnlineScanNav/observables.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-07/2-OnlineScanNav/gps_ephemeris.xml";
#endif

#if 0

    // sym fs 16MHz
    // PPP 3D SD[m] == 30m
    const int error_bound = 200;
    arma::vec true_r_eb_e = { 3655411.386, 1404120.930, 5017902.834};

    // Single 3D SD[m] == 67m
    //const int error_bound = 200;
    // arma::vec true_r_eb_e = { 3655470.883, 1404131.776, 5017938.950};

    int dump_n_channels = 8;
    std::string true_obs_file = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-1/observables.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-1/gps_ephemeris.xml";
#endif

#if 0
    const int error_bound = 200;
    arma::vec true_r_eb_e;

    //4MHz PPP 3D SD[m] = 43.7
    if (USE_PPP)
       true_r_eb_e = arma::vec({ 3655404.400, 1404118.848, 5017895.377});

    //4MHz Single 3D SD[m] = 96
    if (!USE_PPP)
       true_r_eb_e = arma::vec({ 3655475.490, 1404137.966, 5017938.882});

    // current comp
    int dump_n_channels = 8;
    std::string true_obs_file = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-2/observables.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-2/gps_ephemeris.xml";
#endif

#if 0
    //16MHz, PPP, 3D SD[m] = 47.00
    const int error_bound = 200;
    arma::vec true_r_eb_e = { 3655432.602, 1404129.254, 5017900.940};

    int dump_n_channels = 8;
    std::string true_obs_file = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-3/observables.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-3/gps_ephemeris.xml";
#endif

#if 0
    //16MHz, PPP, 3D SD[m] = 49.00
    const int error_bound = 200;
    arma::vec true_r_eb_e = { 3655432.872, 1404130.188, 5017901.024};
    //16MHz, PPP, 3D SD[m] = 27.61
    //const int error_bound = 40;
    //arma::vec true_r_eb_e = { 3655439.9541424359194934, 1404114.2663816625718027, 5017912.8526892913505435};


    int dump_n_channels = 8;
    std::string true_obs_file = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-4/observables.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-4/gps_ephemeris.xml";
#endif

#if 0
    arma::vec true_r_eb_e;
    const int error_bound = 200;

    if(USE_PPP)
    {
        // 4M 3D SD[m] = 43.91
        true_r_eb_e = arma::vec({ 3655403.844, 1404118.503, 5017895.504});
    }
    else
    {
        // 4M 3D SD[m] = 95.65
        true_r_eb_e = arma::vec({ 3655475.8417131230235100, 1404138.1389957570936531, 5017940.8985855309292674});
    }

    int dump_n_channels = 8;
    std::string true_obs_file = "/home/mk/Gnss/Results/2020-04-09/OnlineScanNav-1/observables.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-09/OnlineScanNav-1/gps_ephemeris.xml";
#endif


#if 0
    // current comp
    arma::vec true_r_eb_e;

    //const int error_bound = 200;
    const int error_bound = 10;

    if(USE_PPP)
    {
        // error_bound = 200;
        // true_r_eb_e = arma::vec(
        //    { 3655415.6656871624290943, 1404120.1573056990746409, 5017908.0126902842894197});

        // error_bound = 10
        // 3D RMS[m] = 4.85
        true_r_eb_e = arma::vec(
            {3655426.2032937305048108,1404129.3436893990729004,5017912.1325014652684331});

    }
    else
    {
        true_r_eb_e = arma::vec({ 3655402.8336528479121625, 1404117.1551710774656385, 5017895.3089261008426547});
    }

    int dump_n_channels = 8;
    std::string true_obs_file = "/home/mk/Gnss/Results/2020-04-09/OnlineScanNav-3-plus/observables.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-09/OnlineScanNav-3-plus/gps_ephemeris.xml";
#endif


#if 1
    // current comp
    arma::vec true_r_eb_e;

    const int error_bound = 200;
    //const int error_bound = 10;

    if(USE_PPP)
    {
        // error_bound = 200, dziwna długo stosowana refer.
        true_r_eb_e = arma::vec(
            { 3655424.6915760519914329, 1404127.5425126673653722, 5017908.3038758812472224});

        // gps-sdr-sim ref pos
        true_r_eb_e = arma::vec({ 3655463.659, 1404112.314, 5017924.853 });

        // error_bound = 10
        //true_r_eb_e = arma::vec(
        //        { 3655416.7486569550819695, 1404123.0403552409261465, 5017908.1269250828772783});
        //true_r_eb_e = arma::vec(
        //    {3655426.2032937305048108,1404129.3436893990729004,5017912.1325014652684331});

    }
    else
    {
        true_r_eb_e = arma::vec({ 3655474.9567744643427432, 1404137.4245814934838563, 5017939.5866936203092337});
    }

    int dump_n_channels = 8;
    //std::string true_obs_file = "/home/mk/Gnss/NaviSocRepo/tests/OnlineScanNav/observables.dat";
    std::string true_obs_file = "/home/mk/Gnss/SimpRel/observables.q32.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-05/1/gps_ephemeris.xml";
#endif





    // load ephemeris
    //std::string eph_xml_filename = path + "data/rtklib_test/eph_GPS_L1CA_test1_poprawiony.xml";

    // eph pochodzą z uruchomienia gnss-sdr na pliku z gns-sdr-sim
    //std::string eph_xml_filename = "data/rtklib_test/eeph_gpssim_pw.xml";

    //std::string eph_xml_filename = "/home/mk/Gnss/gnss-sdr-my/gps_ephemeris.xml";

    //std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-05/1/gps_ephemeris.xml";



	std::cout << "Hello World!" << std::endl;

	std::string path = std::string(TEST_PATH);

    //std::unique_ptr<Rtklib_Solver> d_ls_pvt(
    //        new Rtklib_Solver( nchannels, dump_filename, flag_dump_to_file,
    //                           save_to_mat, rtk));
	int nchannels = 8;
	std::string dump_filename = ".rtklib_solver_dump.dat";
	bool flag_dump_to_file = false;
	bool save_to_mat = false;

    std::shared_ptr<Rtklib_Solver> d_ls_pvt = std::make_shared<Rtklib_Solver>(nchannels, dump_filename, flag_dump_to_file, save_to_mat, rtk);

    d_ls_pvt->set_averaging_depth(1);


    Gnss_Sdr_Supl_Client supl_client;

    std::cout << "SUPL: Try read GPS ephemeris from XML file " << eph_xml_filename << std::endl;
    if (supl_client.load_ephemeris_xml(eph_xml_filename) == true)
    {
        std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter;
        for (gps_eph_iter = supl_client.gps_ephemeris_map.cbegin(); gps_eph_iter != supl_client.gps_ephemeris_map.cend(); gps_eph_iter++)
        {
            //int prn = gps_eph_iter->first;
            int prn = gps_eph_iter->second.i_satellite_PRN;
            int toe = gps_eph_iter->second.d_Toe;
            if(find_toe > 0 && gps_eph_iter->second.d_Toe != find_toe)
            {
                std::cout << "SKIP EPH PRN: " <<  prn << " TOE: "<< toe << std::endl;
                continue;
            }
            //std::cout << "SUPL: Read XML Ephemeris for GPS SV " << gps_eph_iter->first << std::endl;
            std::cout << "SUPL: Read XML Ephemeris for GPS SV " << prn << std::endl;

            std::shared_ptr<Gps_Ephemeris> tmp_obj = std::make_shared<Gps_Ephemeris>(gps_eph_iter->second);
            // update/insert new ephemeris record to the global ephemeris map
            d_ls_pvt->gps_ephemeris_map[prn] = *tmp_obj;
        }
    }
    else
    {
        std::cout << "ERROR: SUPL client error reading Ephemeris XML" << std::endl;
    }

#if 0
    std::string iono_xml_filename = "/home/mk/Gnss/gnss-sdr-my/gps_iono.xml";
    if(supl_client.load_iono_xml(iono_xml_filename) == true)
    {
        supl_client.gps_iono.valid = true;
        d_ls_pvt->gps_iono = supl_client.gps_iono;
        std::cout << "SUPL: Read XML IONO loaded" << std::endl;

    }else
    {
        std::cout << "ERROR: SUPL client error reading IONO XML" << std::endl;
    }
#endif

#if 0
    std::string utc_xml_filename = "/home/mk/Gnss/gnss-sdr-my/gps_utc_model.xml";
    if (supl_client.load_utc_xml(utc_xml_filename) == true)
    {
        d_ls_pvt->gps_utc_model = supl_client.gps_utc;
        std::cout << "SUPL: Read XML UTC loaded" << std::endl;

    }
    else
    {
        std::cout << "ERROR: SUPL client error reading UTC XML" << std::endl;
    }
#endif



    int decym = 1;

    Observables_Dump_Reader observables (dump_n_channels);  // 1 extra

    if (!observables.open_obs_file(true_obs_file))
        std::cout << "Failure opening true observables file" << std::endl;

    if (!observables.read_binary_obs())
        std::cout << "Failure reading true tracking dump file." << std::endl;

    //auto nepoch = static_cast<unsigned int>(true_obs_data.num_epochs());
    //std::cout << "Measured observations epochs = " << nepoch << std::endl;


    arma::vec sq_sum_ecef = { 0.0, 0.0, 0.0};
    arma::vec sum_meas_pos_ecef = { 0.0, 0.0, 0.0};
    int sum_num = 0;
    int big_err_num = 0;

    // TODO sel refer.
#if (0)
    //reference position on in WGS84: Lat (deg), Long (deg) , H (m): 30.286502,120.032669,100
    //arma::vec LLH = {30.286502, 120.032669, 100};  //ref position for this scenario
    //arma::vec LLH = {52.21904497408158, 21.01267815632622, 168.05925633106381};
    //arma::vec LLH = {52.21904661779532, 21.01268096018381, 168.24609463661909};
    //arma::vec LLH = { 52.21898588275369, 21.01258906723609, 191.01122819073498 }; // 2014-12-20T00:01:00.000Z
    arma::vec LLH = { 52.21898588275369 + 0.0000129838958500, 21.01258906723609 - 0.0000129838958500, 191.01122819073498 - 0.3817213643342257 };

    arma::vec v_eb_n = {0.0, 0.0, 0.0};
    arma::vec true_r_eb_e;
    arma::vec true_v_eb_e;
    pv_Geo_to_ECEF(degtorad(LLH(0)), degtorad(LLH(1)), LLH(2), v_eb_n, true_r_eb_e, true_v_eb_e);
#else
    // gps-sdr-sim ref pos
    //arma::vec true_r_eb_e = { 3655463.659, 1404112.314, 5017924.853 };

    //arma::vec true_r_eb_e = { 3655449.287, 1404116.477, 5017919.645};
    //arma::vec true_r_eb_e = { 3655449.316, 1404116.447, 5017919.394};

    //arma::vec true_r_eb_e = { 3655448.789, 1404116.414, 5017919.164};

    // gps-sdr-sim + gnss-sdr, elev_mask=0, no decym
    //arma::vec true_r_eb_e = { 3655430.929, 1404117.744, 5017912.359} ;

    //mean >> /2020-04-07/2-OnlineScanNav
    //arma::vec true_r_eb_e = { 3655482.807, 1404118.578, 5017917.000};
    //arma::vec true_r_eb_e = { 3655472.0608506547287107, 1404122.9145570218097419, 5017929.7938752910122275};

    //16M PPP
    //arma::vec true_r_eb_e = { 3655411.386, 1404120.930, 5017902.834};

    // 16M single
    //arma::vec true_r_eb_e = { 3655470.884, 1404131.776, 5017938.950};

    //arma::vec true_r_eb_e = { 3655479.590, 1404123.428, 5017946.443};



    arma::vec LLH =  LLH_to_deg(cart2geo(true_r_eb_e, 4));
#endif


    std::size_t dirPos = true_obs_file.find_last_of("/");
    std::string gpxDir = true_obs_file.substr(0, dirPos);
    std::string gpxFName = true_obs_file.substr(dirPos + 1, true_obs_file.length());

    std::cout << "gpx dir: " << gpxDir << std::endl;
    std::cout << "gpx fname: " << gpxFName << std::endl;

    Gpx_Printer gpx_dump(gpxDir);
    gpx_dump.set_headers(gpxFName);


    FILE *diffCsv = fopen((true_obs_file + "_diff.csv").c_str(), "w");
    fprintf(diffCsv, "time; diff[m]; gdop\n");

    int64_t epoch_counter = 0;
    int time_epoch = 0;
    double prev_time = -1;
    //int chan = 0;

    observables.restart();
    while (observables.read_binary_obs())
    {
        std::map<int, Gnss_Synchro> gnss_synchro_map;

        double rx_time = 0;
        bool anyValid = false;

        for (int n = 0; n < dump_n_channels; n++)
        {
            //std::cout << "prn=" << *true_obs_data.PRN << std::endl;
            //std::cout << "tow=" << *true_obs_data.TOW_at_current_symbol_s << std::endl;

            bool valid = static_cast<bool>(observables.valid[n]);


            //if (valid)
            {
                Gnss_Synchro gns_syn;
                gns_syn.System = 'G';
                gns_syn.Signal[0] = '1';
                gns_syn.Signal[1] = 'C';
                gns_syn.Flag_valid_word = valid;
                gns_syn.RX_time = observables.RX_time[n];
                gns_syn.interp_TOW_ms = observables.TOW_at_current_symbol_s[n] * 1000;
                gns_syn.Carrier_Doppler_hz = observables.Carrier_Doppler_hz[n];
                gns_syn.Carrier_phase_rads = observables.Acc_carrier_phase_hz[n] * GPS_TWO_PI;
                gns_syn.Pseudorange_m = observables.Pseudorange_m[n];
                gns_syn.PRN = observables.PRN[n];

                // src/algorithms/observables/gnuradio_blocks/hybrid_observables_gs.cc
                //    tmp_double = out[i][0].RX_time;
                //    tmp_double = out[i][0].interp_TOW_ms / 1000.0;
                //    tmp_double = out[i][0].Carrier_Doppler_hz;
                //    tmp_double = out[i][0].Carrier_phase_rads / GPS_TWO_PI;
                //    tmp_double = out[i][0].Pseudorange_m;
                //    tmp_double = static_cast<double>(out[i][0].PRN);
                //    tmp_double = static_cast<double>(out[i][0].Flag_valid_pseudorange);

                gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(n, gns_syn));

                if (0 && epoch_counter > 4900 && epoch_counter < 4910)
                {
                    if (gns_syn.RX_time - prev_time > 1e-6)
                        std::cout << "***" << std::endl;

                    std::cout << std::setprecision(12);
                    std::cout << std::endl;
                    std::cout << "  valid = " << valid << std::endl;
                    std::cout << "    PRN = " << (int)observables.PRN[n] << std::endl;
                    std::cout << "RX_time = " << observables.RX_time[n] << std::endl;
                    std::cout << " TOW_ms = " << observables.TOW_at_current_symbol_s[n] << std::endl;
                    std::cout << "p-range = " << observables.Pseudorange_m[n] << std::endl;
                    if (n == dump_n_channels - 1)
                        std::cout << "---" << std::endl;
                }

                if (valid)
                {
                    rx_time = gns_syn.RX_time;
                }

                if (gns_syn.RX_time - prev_time > 1e-6)
                {
                    time_epoch++;
                    prev_time = gns_syn.RX_time;
                }

            }

            anyValid = anyValid || valid;
        }

        //chan ++;
        //if(chan==dump_chan_num)
        //    chan = 0;

        epoch_counter++;

        if (!anyValid)
            continue;

        if (time_epoch % decym)
            continue;

        // if (epoch_counter < 20)
        //    continue;

        if (d_ls_pvt->get_PVT(gnss_synchro_map, false))
            {

                // DEBUG MESSAGE: Display position in console output
                if (d_ls_pvt->is_valid_position())
                    {
                        std::streamsize ss = std::cout.precision();  // save current precision
                        std::cout.setf(std::ios::fixed, std::ios::floatfield);

                        auto facet = new boost::posix_time::time_facet("%Y-%b-%d %H:%M:%S.%f %z");
                        std::cout.imbue(std::locale(std::cout.getloc(), facet));

                        std::cout << "Position at " << d_ls_pvt->get_position_UTC_time()
                                  << " UTC using " << d_ls_pvt->get_num_valid_observations()
                                  << std::fixed << std::setprecision(9)
                                  << " observations is Lat = " << d_ls_pvt->get_latitude() << " [deg], Long = " << d_ls_pvt->get_longitude()
                                  << std::fixed << std::setprecision(3)
                                  << " [deg], Height = " << d_ls_pvt->get_height() << " [m]" << std::endl;
                        std::cout << std::setprecision(ss);
                        std::cout << "RX clock offset: " << d_ls_pvt->get_time_offset_s() << "[s]" << std::endl;

                        // boost::posix_time::ptime p_time;
                        // gtime_t rtklib_utc_time = gpst2time(adjgpsweek(d_ls_pvt->gps_ephemeris_map.cbegin()->second.i_GPS_week), d_rx_time);
                        // p_time = boost::posix_time::from_time_t(rtklib_utc_time.time);
                        // p_time += boost::posix_time::microseconds(round(rtklib_utc_time.sec * 1e6));
                        // std::cout << TEXT_MAGENTA << "Observable RX time (GPST) " << boost::posix_time::to_simple_string(p_time) << TEXT_RESET << std::endl;

                        std::cout << "RTKLIB Position at RX TOW = " << rx_time
                                  << " in ECEF (X,Y,Z,t[meters]) = " << std::fixed << std::setprecision(16)
                                  << d_ls_pvt->pvt_sol.rr[0] << ","
                                  << d_ls_pvt->pvt_sol.rr[1] << ","
                                  << d_ls_pvt->pvt_sol.rr[2] << std::endl;
                        /* std::cout << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_ls_pvt->get_position_UTC_time())
                                 << " UTC using "<< d_ls_pvt->get_num_valid_observations() <<" observations is HDOP = " << d_ls_pvt->get_hdop() << " VDOP = "
                                 << d_ls_pvt->get_vdop()
                                 << " GDOP = " << d_ls_pvt->get_gdop() << std::endl; */


                        double error_LLH_m = great_circle_distance(LLH(0), LLH(1), d_ls_pvt->get_latitude(), d_ls_pvt->get_longitude());
                        std::cout << "Haversine Great Circle error LLH distance: " << error_LLH_m << " [meters]" << std::endl;

                        arma::vec measured_r_eb_e = {d_ls_pvt->pvt_sol.rr[0], d_ls_pvt->pvt_sol.rr[1], d_ls_pvt->pvt_sol.rr[2]};

                        arma::vec error_r_eb_e = measured_r_eb_e - true_r_eb_e;

                        // std::cout << "ECEF position error vector: " << error_r_eb_e << " [meters]" << std::endl;

                        double error_3d_m = arma::norm(error_r_eb_e, 2);

                        std::cout << "3D positioning error: " << error_3d_m << " [meters]" << std::endl;

                        if (error_3d_m >= error_bound) //200.0)
                        {
                            std::cout << "3D positioning error BIG!" << std::endl;
                            big_err_num++;
                        }
                        else
                        {
                            std::cout << "3D positioning error OK!" << std::endl;
                            sq_sum_ecef = sq_sum_ecef + arma::pow(error_r_eb_e, 2);
                            sum_meas_pos_ecef = sum_meas_pos_ecef + measured_r_eb_e;
                            sum_num++;
                        }

                        fprintf(diffCsv, "%.12g; %g; %g\n", rx_time, error_3d_m, d_ls_pvt->get_gdop());

                    }else
                    {
                        std::cout << "not valid" << std::endl;
                    }
            }else
            {
                std::cout << "not comp" << std::endl;
            }



        gpx_dump.print_position(d_ls_pvt, false);

    }

    std::cout << "-----------------" << std::endl;
    std::cout << "USE_PPP = " << USE_PPP << std::endl;
    std::cout << "mean meas num = " << sum_num <<std::endl;
    std::cout << "big_err_num = " << big_err_num <<std::endl;

    if(sum_num)
    {
        std::cout << "3D RMS[m] = " << sqrt(arma::sum(sq_sum_ecef / sum_num)) << std::endl;
        arma::vec mean_pos = sum_meas_pos_ecef / sum_num;

        std::cout << "ref ECEF position: { " << true_r_eb_e(0) << ", " << true_r_eb_e(1) << ", " << true_r_eb_e(2) << "}; " << std::endl;
        std::cout << "mean meas ECEF position: { " << mean_pos(0) << ", " << mean_pos(1) << ", " << mean_pos(2) << "}; " << std::endl;
        std::cout << "mean - ref [m] = " << arma::norm(mean_pos - true_r_eb_e, 2) << std::endl;
    }

    std::cout << "epoch_counter = " << epoch_counter << std::endl;
    std::cout << "time_epoch    = " << time_epoch << std::endl;

    std::cout << "DONE" << std::endl;


    // solve
    //bool pvt_valid = false;

    fclose(diffCsv);
    diffCsv = 0;

	return 0;
}

