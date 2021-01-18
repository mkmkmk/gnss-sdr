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

#include "observables_dump_reader.h"

#include "gpx_printer.h"

#include <stdio.h>
#include <assert.h>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/map.hpp>

#include "file_configuration.h"


#define LOG(severity) std::cout

#define WRITE_OBS_CSV (1)

// próg realnego biasu
#define REAL_BIAS_THRESH (5000)

// długoś uśredniania średniego biasu
#define BIAS_SMOOTH_MEAN_N (250)

// -- ad usuwanie odstających biasu częstotliwości
// liczba kolejnych odstających od ogólnej średniej po której kolejne przestają być odstającymi
#define CARR_BIAS_CMP_MAX_SKIP (5)

// próg w Hz na odstających z ogólnej średniej biasu
#define CARR_BIAS_OUT_THRESH (100)


#define LAMBDA_L1 ( SPEED_OF_LIGHT_M_S / 1540 / 1023000)
#define LAMBDA_L5 ( SPEED_OF_LIGHT_M_S / 1150 / 1023000)
#define LAMBDA_XX(IS_DUAL) ((IS_DUAL) ? LAMBDA_L5 : LAMBDA_L1)

#define IS_DUAL(ARG) ((ARG) & 0x80)


//TODO Single vs PPP
//#define USE_PPP (1)

std::string positioning_mode_str;



template <size_t N>
class MovingAv
{
private:
    double samples_[N];
    size_t num_samples_{0};
    double total_{0};

public:
    double next(double sample)
    {
        if (num_samples_ < N)
        {
            samples_[num_samples_++] = sample;
            total_ += sample;
        }
        else
        {
            double& oldest = samples_[num_samples_++ % N];
            total_ += sample - oldest;
            oldest = sample;
        }
        return total_ / std::min(num_samples_, N);
    }


};



rtk_t configure_rtklib_options(std::shared_ptr<FileConfiguration> configuration)
{
    std::string role = "rtklib_solver";

    /*
    positioning_mode=PPP_Static  ; options: Single, Static, Kinematic, PPP_Static, PPP_Kinematic
    iono_model=Broadcast ; options: OFF, Broadcast, SBAS, Iono-Free-LC, Estimate_STEC, IONEX
    PVT.trop_model=Saastamoinen ; options: OFF, Saastamoinen, SBAS, Estimate_ZTD, Estimate_ZTD_Grad
    */
#if 0
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

        //configuration->set_property("rtklib_solver.iono_model", "OFF");
        configuration->set_property("rtklib_solver.trop_model", "OFF");
        configuration->set_property("rtklib_solver.iono_model", "Broadcast");
        //configuration->set_property("rtklib_solver.trop_model", "Saastamoinen");
    }
    //RTKLIB PVT solver options
#endif

    // Settings 1
    int positioning_mode = -1;
    std::string default_pos_mode("Single");

    //configuration->set_property(role + ".positioning_mode", default_pos_mode);

    positioning_mode_str = configuration->property(role + ".positioning_mode", default_pos_mode); /* (PMODE_XXX) see src/algorithms/libs/rtklib/rtklib.h */
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


std::map<int, Gps_Ephemeris> load_gps_ephemeris(std::string eph_xml_filename, int tow)
{
    std::map<int, Gps_Ephemeris> gps_ephemeris_map;
    Gnss_Sdr_Supl_Client supl_client;

    std::cout << "SUPL: Try read GPS ephemeris from XML file " << eph_xml_filename << std::endl;
    if (supl_client.load_ephemeris_xml(eph_xml_filename) == true)
    {
        //std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter;
        for (auto gps_eph_iter = supl_client.gps_ephemeris_map.cbegin(); gps_eph_iter != supl_client.gps_ephemeris_map.cend(); gps_eph_iter++)
        {
            //int prn = gps_eph_iter->first;
            int prn = gps_eph_iter->second.i_satellite_PRN;
            int toe = gps_eph_iter->second.d_Toe;
            int week = gps_eph_iter->second.i_GPS_week;

            //if (find_toe > 0 && gps_eph_iter->second.d_Toe != find_toe)
            //TODO trzeba będzie jakoś na bieżąco uaktualniać eph
            //if (tow > 0 && abs(gps_eph_iter->second.d_Toe - tow) > 60)
            if (tow > 0)
            {
                if (tow < toe || (gps_ephemeris_map.count(prn) && gps_ephemeris_map[prn].d_Toe > toe))
                {
                    //std::cout << "SKIP EPH PRN: " << prn << " TOE: " << toe << " week: " << week << std::endl;
                    continue;
                }
            }
            //int min = -1;
            // for(auto it = gps_ephemeris_map.find(prn); it != gps_ephemeris_map.end(); it++)

            std::cout << "SUPL: Read XML Ephemeris for GPS SV " << prn << " TOE: "<< toe << " week: " << week << std::endl;

            std::shared_ptr<Gps_Ephemeris> tmp_obj = std::make_shared<Gps_Ephemeris>(gps_eph_iter->second);
            // update/insert new ephemeris record to the global ephemeris map
            gps_ephemeris_map[prn] = *tmp_obj;
        }
    }
    else
    {
        std::cout << "ERROR: SUPL client error reading Ephemeris XML" << std::endl;
    }
    return gps_ephemeris_map;
}

std::map<int, Galileo_Ephemeris> load_gal_ephemeris(std::string eph_xml_filename, int tow)
{
    std::map<int, Galileo_Ephemeris> ephemeris_map;
    Gnss_Sdr_Supl_Client supl_client;

    std::cout << "SUPL: Try read GALILEO ephemeris from XML file " << eph_xml_filename << std::endl;
    if (supl_client.load_gal_ephemeris_xml(eph_xml_filename) == true)
    {
        //std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter;
        for (auto eph_iter = supl_client.gal_ephemeris_map.cbegin(); eph_iter != supl_client.gal_ephemeris_map.cend(); eph_iter++)
        {
            //int prn = gps_eph_iter->first;
            int prn = eph_iter->second.i_satellite_PRN;
            int toe = eph_iter->second.t0e_1;
            int week = eph_iter->second.WN_5;

            //if (find_toe > 0 && gps_eph_iter->second.d_Toe != find_toe)
            //TODO trzeba będzie jakoś na bieżąco uaktualniać eph
            //if (tow > 0 && abs(gps_eph_iter->second.d_Toe - tow) > 60)
            if (tow > 0)
            {
                if (tow < toe || (ephemeris_map.count(prn) && ephemeris_map[prn].t0e_1 > toe))
                {
                    //std::cout << "SKIP EPH PRN: " << prn << " TOE: " << toe << " week: " << week << std::endl;
                    continue;
                }
            }
            //int min = -1;
            // for(auto it = gps_ephemeris_map.find(prn); it != gps_ephemeris_map.end(); it++)

            std::cout << "SUPL: Read XML Ephemeris for GALILEO SV " << prn << " TOE: "<< toe << " week: " << week << std::endl;

            auto tmp_obj = std::make_shared<Galileo_Ephemeris>(eph_iter->second);
            // update/insert new ephemeris record to the global ephemeris map
            ephemeris_map[prn] = *tmp_obj;
        }
    }
    else
    {
        std::cout << "ERROR: SUPL client error reading Ephemeris XML" << std::endl;
    }
    return ephemeris_map;
}


void save_ephemeris_csv(std::string eph_xml_filename)
{
    
    Gnss_Sdr_Supl_Client supl_client;
    if (supl_client.load_ephemeris_xml(eph_xml_filename) == false)
    {
        std::cout << "ERROR: SUPL client error reading Ephemeris XML" << std::endl;
        return; 
    }    

    FILE *fcsv = fopen((eph_xml_filename + ".csv").c_str(), "w");
    if (fcsv == NULL)
    {
        printf("WRITE FILE OPEN FAILED \n");
        return;
    }
    fprintf(fcsv, "i_satellite_PRN, d_TOW, d_IODE_SF2, d_IODE_SF3, d_Crs, d_Delta_n, d_M_0, d_Cuc, d_e_eccentricity, d_Cus, d_sqrt_A, d_Toe, d_Toc, d_Cic-, d_OMEGA0, d_Cis, d_i_0, d_Crc, d_OMEGA, d_OMEGA_DOT-, d_IDOT, i_code_on_L2, i_GPS_week, b_L2_P_data_flag, i_SV_accuracy, i_SV_health, d_TGD, d_IODC, i_AODO, b_fit_interval_flag, d_spare1, d_spare2, d_A_f0-, d_A_f1, d_A_f2, b_integrity_status_flag, b_alert_flag, b_antispoofing_flag\n");

    std::map<int, Gps_Ephemeris>::const_iterator gps_eph_iter;
    for (gps_eph_iter = supl_client.gps_ephemeris_map.cbegin(); gps_eph_iter != supl_client.gps_ephemeris_map.cend(); gps_eph_iter++)
    {
        Gps_Ephemeris e = gps_eph_iter->second;
        fprintf(
            fcsv, 
            "%d, %d, %d, %d, %30.20g, %30.20g, %30.20g, %30.20g, %30.20g, %30.20g, %30.20g, %d, %d, %30.20g, %30.20g, %30.20g, %30.20g, %30.20g, %30.20g, %30.20g, %30.20g, %d, %d, %d, %d, %d, %30.20g, %d, %d, %d, %30.20g, %30.20g, %30.20g, %30.20g, %30.20g, %d, %d, %d\n", 
            e.i_satellite_PRN,
	    e.d_TOW,
	    e.d_IODE_SF2,
	    e.d_IODE_SF3,
	    e.d_Crs,
	    e.d_Delta_n,
	    e.d_M_0,
	    e.d_Cuc,
	    e.d_e_eccentricity,
	    e.d_Cus,
	    e.d_sqrt_A,
	    e.d_Toe,
	    e.d_Toc,
	    e.d_Cic,
	    e.d_OMEGA0,
	    e.d_Cis,
	    e.d_i_0,
	    e.d_Crc,
	    e.d_OMEGA,
	    e.d_OMEGA_DOT,
	    e.d_IDOT,
	    e.i_code_on_L2,
	    e.i_GPS_week,
	    e.b_L2_P_data_flag,
	    e.i_SV_accuracy,
	    e.i_SV_health,
	    e.d_TGD,
	    e.d_IODC,
	    e.i_AODO,
	    e.b_fit_interval_flag,
	    e.d_spare1,
	    e.d_spare2,
	    e.d_A_f0,
	    e.d_A_f1,
	    e.d_A_f2,
	    e.b_integrity_status_flag,
	    e.b_alert_flag,
	    e.b_antispoofing_flag
            
            );

    }
    
    fclose(fcsv);
}



void write_obs_csv(FILE *fcsv, const Gnss_Synchro *o, double *prev_tm, double *prev_carr, double *prev_rg, std::shared_ptr<MovingAv<50>> *bias_smth)
{
    double ttime  = o->Pseudorange_m / SPEED_OF_LIGHT_M_S;// - .068;
    double carr = o->Carrier_phase_rads / TWO_PI;

    double tm_dt = o->RX_time - *prev_tm;
    double carr_f = 0;
    double range_f = 0;
    double bias_f = 0;

    int isDual = o->Signal == std::string("L5") || o->Signal == std::string("5X");

    if (*prev_tm >= 0.001 && tm_dt >= 0.001)
    {
        carr_f = -(carr - *prev_carr) / tm_dt;
        range_f = -(o->Pseudorange_m - *prev_rg) / tm_dt / LAMBDA_XX(isDual);

        if(fabs(carr_f-range_f) < 5000)
            bias_f = (*bias_smth)->next(carr_f - range_f);
        else
        {
            printf("*** freq bias comp ovf carr_fq = %g rg_fq=%g\n", carr_f, range_f);
        }
    }
    else
    {
        tm_dt = 0.001;
    }


    fprintf(
            fcsv,
            "%.15g; %.12g; %.20g; %d; %g; %.15g; %g; %g; %g\n",
            o->RX_time,
            o->Pseudorange_m,
            carr,
            o->PRN,
            o->Carrier_Doppler_hz,
            ttime,
            carr_f,
            range_f,
            bias_f
    );
    *prev_carr = carr;
    *prev_tm = o->RX_time;
    *prev_rg = o->Pseudorange_m;
}


int start_obs_csv(const char *readPath, FILE **fcsv_ch, int chan_num)
{
    char strBuf[1000];

    printf("Out files:\n");
    for (int i = 0; i < chan_num; ++i)
    {
        sprintf(strBuf, "%s_ch%d.csv", readPath, i);
        printf("  %s\n", strBuf);

        //rx_times[i] = -1;
        //rx_ofs[i] = 0;
        fcsv_ch[i] = fopen(strBuf, "wb");
        if (fcsv_ch[i] == NULL)
        {
            printf("write file open error\n");
            return 1;
        }
        // fprintf(fcsv_ch[i], "rx_time; p-rng_ch%d; phase_cyc_ch%d; prn_ch%d; valid_ch%d\n", i, i, i, i);
        fprintf(fcsv_ch[i], "rx_time; p-rng_ch%d; phase_cyc_ch%d; prn_ch%d; doppler; t-time; dopp(carr); dopp(rng); bias\n", i, i, i);

    }
    printf("--\n");
    return 0;
}


void close_obs_csv(FILE **fcsv_ch, int chan_num)
{
    for (int i = 0; i < chan_num; ++i)
    {
         fclose(fcsv_ch[i]);
         fcsv_ch[i] = 0;
    }
}


double comp_carr_bias_pre_flt(double *curr_carr_biases, int curr_carr_biases_num)
{
    double carr_bias_pre_flt;

    if (0)
    {
        std::vector<double> curr_carr_biases_v(curr_carr_biases, curr_carr_biases + curr_carr_biases_num);
        std::sort(curr_carr_biases_v.begin(), curr_carr_biases_v.end());
        carr_bias_pre_flt = curr_carr_biases_v[curr_carr_biases_num / 2];
    }

    double bsum0 = 0;
    for (int ii = 0; ii < curr_carr_biases_num; ++ii)
        bsum0 += curr_carr_biases[ii];
    carr_bias_pre_flt = bsum0 / curr_carr_biases_num;

    if (curr_carr_biases_num > 1)
    {
        double bsum1 = 0;
        int num1 = 0;
        for (int ii = 0; ii < curr_carr_biases_num; ++ii)
        {
            if (fabs(curr_carr_biases[ii] - bsum0) > CARR_BIAS_OUT_THRESH)
                continue;
            bsum1 += curr_carr_biases[ii];
            num1++;
        }
        if (num1 != curr_carr_biases_num && num1 > 0)
            carr_bias_pre_flt = bsum1 / num1;
    }

    return carr_bias_pre_flt;
}


void check_gps_eph(const std::map<int, Gnss_Synchro>& gnss_synchro_map, std::map<int, Gps_Ephemeris>& gps_ephemeris_map, double rx_time)
{
    for (auto it = gnss_synchro_map.cbegin(); it != gnss_synchro_map.cend(); it++)
    {
        auto fnd = gps_ephemeris_map.find(it->second.PRN);
        if (fnd == gps_ephemeris_map.end())
        {
            std::cout << "*** MISSING EPH FOR PRN " << it->second.PRN << std::endl;
            //continue;
        }
    }
    for (auto it = gps_ephemeris_map.cbegin(); it != gps_ephemeris_map.cend(); it++)
    {
        double toe = it->second.d_Toe;
        if (rx_time - toe > 7200)
        {
            std::cout << "*** OLD Ephemeris for GPS SV " << it->second.i_satellite_PRN << " TOW: " << rx_time << " TOE: " << toe << " DIFF: " << (rx_time - toe) << " s "
                    << std::endl;
        }
        break;
    }
}


void check_gal_eph(const std::map<int, Gnss_Synchro>& gnss_synchro_map,  std::map<int, Galileo_Ephemeris>& galileo_ephemeris_map, double rx_time)
{
    for (auto it = gnss_synchro_map.cbegin(); it != gnss_synchro_map.cend(); it++)
    {
        auto fnd = galileo_ephemeris_map.find(it->second.PRN);
        if (fnd == galileo_ephemeris_map.end())
        {
            std::cout << "*** MISSING EPH FOR PRN " << it->second.PRN << std::endl;
            //continue;
        }
    }
    for (auto it = galileo_ephemeris_map.cbegin(); it != galileo_ephemeris_map.cend(); it++)
    {
        double toe = it->second.t0e_1;
        if (rx_time - toe > 7200)
        {
            std::cout << "*** OLD Ephemeris for GPS SV " << it->second.i_satellite_PRN << " TOW: " << rx_time << " TOE: " << toe << " DIFF: " << (rx_time - toe) << " s "
                    << std::endl;
        }
        break;
    }
}


int main(int argc, char** argv)
{

    if(argc < 2)
    {
        printf("arg: conf path \n");
        return 1;
    }

    std::string conf_path(argv[1]);

    std::cout << "conf_path = " << conf_path << std::endl;

    //std::shared_ptr<InMemoryConfiguration> configuration;
    //configuration = std::make_shared<InMemoryConfiguration>();

    std::shared_ptr<FileConfiguration> configuration(new FileConfiguration(conf_path));

    std::string obs_filename = configuration->property("obs_to_nav.obs_filename", std::string(""));
    int obs_n_channels = configuration->property("obs_to_nav.obs_n_channels", 1);

    std::string eph_xml_filename = configuration->property("obs_to_nav.eph_xml_filename", std::string(""));
    std::string iono_xml_filename = configuration->property("obs_to_nav.iono_xml_filename", std::string(""));
    std::string utc_xml_filename = configuration->property("obs_to_nav.utc_model_xml_filename", std::string(""));

    double ref_ecef_X = configuration->property("obs_to_nav.ref_ecef_X", 0.0);
    double ref_ecef_Y = configuration->property("obs_to_nav.ref_ecef_Y", 0.0);
    double ref_ecef_Z = configuration->property("obs_to_nav.ref_ecef_Z", 0.0);

    double error_bound = configuration->property("obs_to_nav.big_error_bound", 1.0);

    arma::vec true_r_eb_e = arma::vec({ ref_ecef_X, ref_ecef_Y, ref_ecef_Z });

    bool save_gpx = configuration->property("obs_to_nav.save_gpx", false);

    double carr_bias0 = configuration->property("obs_to_nav.carr_bias", 0.0);
    std::cout << "carr_bias = " << carr_bias0 << std::endl;
    double carr_bias_cmp = carr_bias0;
    double carr_bias2_cmp = carr_bias0;
    double carr_bias_pre_flt = 0;
    double carr_bias2_pre_ft = 0;
    int carr_bias_cmp_skip = 0;
    int carr_bias2_cmp_skip = 0;

    // usuwanie lagu, ile próbek w przyszłoś patrzeć
    int future_bias_lag = configuration->property("obs_to_nav.future_bias_lag", 75);

    int rem_comp_bias = configuration->property("obs_to_nav.rem_comp_bias", 1);

#if 0
#include "conf-inc-obso.c"
#endif

    std::string dump_filename = ".rtklib_solver_dump.dat";
    bool flag_dump_to_file = false;
    bool save_to_mat = false;

    rtk_t rtk = configure_rtklib_options(configuration);

    int isGalileo = 0;

    if (rtk.opt.navsys == 1)
    {
        //isGalileo = 0;
        isGalileo = MK_MOD_GPS_AS_GALILEO;
    }
    else if (rtk.opt.navsys == 8)
    {
        isGalileo = 1;
    }
    else
    {
        printf("not implemented GNSS system : '%d' \n", rtk.opt.navsys);
        return 1;
    }

    if (!isGalileo)
        save_ephemeris_csv(eph_xml_filename);

    //std::unique_ptr<Rtklib_Solver> d_ls_pvt(
    //        new Rtklib_Solver( nchannels, dump_filename, flag_dump_to_file,
    //                           save_to_mat, rtk));

    auto d_ls_pvt = std::make_shared<Rtklib_Solver>(rtk, obs_n_channels, dump_filename, flag_dump_to_file, save_to_mat);

    d_ls_pvt->set_averaging_depth(1);

    Gnss_Sdr_Supl_Client supl_client;

    if (supl_client.load_iono_xml(iono_xml_filename) == true)
    {
        supl_client.gps_iono.valid = true;
        d_ls_pvt->gps_iono = supl_client.gps_iono;
        std::cout << "SUPL: Read XML IONO loaded" << std::endl;

    }
    else
    {
        std::cout << "ERROR: SUPL client error reading IONO XML" << std::endl;
    }

    if (supl_client.load_utc_xml(utc_xml_filename) == true)
    {
        d_ls_pvt->gps_utc_model = supl_client.gps_utc;
        std::cout << "SUPL: Read XML UTC loaded" << std::endl;
    }
    else
    {
        std::cout << "ERROR: SUPL client error reading UTC XML" << std::endl;
    }


    int decym = 1;

    Observables_Dump_Reader observables(obs_n_channels);  // 1 extra
    Observables_Dump_Reader fu_observables(obs_n_channels); // przyszłe


    if (!observables.open_obs_file(obs_filename))
        std::cout << "Failure opening true observables file" << std::endl;

    if (!observables.read_binary_obs())
        std::cout << "Failure reading true tracking dump file." << std::endl;

    if (!fu_observables.open_obs_file(obs_filename))
        std::cout << "Failure opening true observables file" << std::endl;


    arma::vec sq_sum_ecef = { 0.0, 0.0, 0.0};
    arma::vec sum_meas_pos_ecef = { 0.0, 0.0, 0.0};
    int sum_num = 0;
    int big_err_num = 0;

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

    arma::vec LLH =  LLH_to_deg(cart2geo(true_r_eb_e, 4));

#endif

    std::size_t dirPos = obs_filename.find_last_of("/");
    if (dirPos == std::string::npos)
        dirPos = 0;

    std::string gpxDir =
            dirPos ?
                    obs_filename.substr(0, dirPos) :
                    ".";

    std::string gpxFName =
            dirPos ?
              obs_filename.substr(dirPos + 1, obs_filename.length()) :
              obs_filename;

    if (save_gpx)
    {
        std::cout << "gpx dir: " << gpxDir << std::endl;
        std::cout << "gpx fname: " << gpxFName << std::endl;
    }
    else
    {
        std::cout << "gpx save OFF" << std::endl;
    }

    Gpx_Printer gpx_dump(gpxDir);
    if (save_gpx)
        gpx_dump.set_headers(gpxFName);


    FILE *diffCsv = fopen((obs_filename + "_diff.csv").c_str(), "w");
    fprintf(diffCsv, "time; diff_3D [m]; diff_2D [m]; gdop; max_abs_resp[m]; max_abs_resc[m]; bias[Hz]; bias_no_flt[Hz]; bias2[Hz]\n");

    FILE *resCsv = fopen((obs_filename + "_res.csv").c_str(), "w");
    fprintf(resCsv, "time");
    for (int i = 0; i < obs_n_channels; i++)
        fprintf(resCsv, "; resRng%d; resCarr%d", i, i);
    fprintf(resCsv, "\n");


    int64_t epoch_counter = 0;
    int time_epoch = 0;
    double prev_time = -1;
    //int chan = 0;

    rtkopenstat((obs_filename + "_stat.txt").c_str(), 2);

    FILE *fcsv_ch[obs_n_channels];

    double prev_csv_carr[obs_n_channels] = {0};
    double prev_csv_rg[obs_n_channels] = {0};
    double prev_csv_tm[obs_n_channels] = {0};

    double carr_acc[obs_n_channels] = {0};

    double prev_rxtime[obs_n_channels] = {0};
    double prev_carr[obs_n_channels] = {0};
    double prev_range[obs_n_channels] = {0};

    double fu_prev_rxtime[obs_n_channels] = {0};
    double fu_prev_carr[obs_n_channels] = {0};
    double fu_prev_range[obs_n_channels] = {0};

    std::shared_ptr<MovingAv<50>> bias_csv_smth[obs_n_channels];

    auto bias_smth = std::make_shared<MovingAv<BIAS_SMOOTH_MEAN_N>>();
    auto bias2_smth = std::make_shared<MovingAv<BIAS_SMOOTH_MEAN_N>>();

    std::shared_ptr<MovingAv<50>> carr_smth[obs_n_channels];
    std::shared_ptr<MovingAv<50>> rng_smth[obs_n_channels];
    std::shared_ptr<MovingAv<50>> rx_smth[obs_n_channels];
    for (int i = 0; i < obs_n_channels; ++i)
    {
        carr_smth[i] = std::make_shared<MovingAv<50>>();
        rng_smth[i] = std::make_shared<MovingAv<50>>();
        rx_smth[i] = std::make_shared<MovingAv<50>>();
        bias_csv_smth[i] = std::make_shared<MovingAv<50>>();
    }

    if (WRITE_OBS_CSV)
        start_obs_csv(obs_filename.c_str(), fcsv_ch, obs_n_channels);

    int last_eph_update_tm = -1;

    int fu_lag_startup = future_bias_lag;

    observables.restart();
    //while (observables.read_binary_obs())
    while (1)
    {
        double curr_carr_biases[obs_n_channels];
        int curr_carr_biases_num = 0;

        double curr_carr_biases2[obs_n_channels];
        int curr_carr_biases2_num = 0;

        if (fu_observables.read_binary_obs())
        {
            for (int n = 0; n < obs_n_channels; n++)
            {
                bool valid = static_cast<bool>(fu_observables.valid[n]);
                if (!valid)
                    continue;

                int prn = observables.PRN[n];

                //if (IS_DUAL(prn))
                //    continue;

                double carr = fu_observables.Acc_carrier_phase_hz[n];
                double range = fu_observables.Pseudorange_m[n];
                double tm_dt = fu_observables.RX_time[n] - fu_prev_rxtime[n];
                if (fu_prev_rxtime[n] >= 0.001 && tm_dt >= 0.001)
                {
                    double carr_f = -(carr - fu_prev_carr[n]) / tm_dt;
                    double range_f = -(range - fu_prev_range[n]) / tm_dt / LAMBDA_XX(IS_DUAL(prn));


                    if (fabs(carr_f - carr_bias0 - range_f) < REAL_BIAS_THRESH)
                    {
                        //carr_bias_cmp = bias_smth->next(carr_f - range_f);
                        if (!IS_DUAL(prn))
                        {
                            curr_carr_biases[curr_carr_biases_num] = carr_f - range_f;
                            curr_carr_biases_num++;
                        }
                        else
                        {
                            curr_carr_biases2[curr_carr_biases2_num] = carr_f - range_f;
                            curr_carr_biases2_num++;
                        }
                    }
                    else
                    {
                        printf("*** freq bias comp ovf carr_fq = %g rg_fq=%g\n", carr_f, range_f);
                    }
                }
                fu_prev_rxtime[n] = fu_observables.RX_time[n];
                fu_prev_carr[n] = carr;
                fu_prev_range[n] = range;
            }

        }

        // comp mean carr bias
        if (rem_comp_bias && curr_carr_biases_num)
        {
            carr_bias_pre_flt = comp_carr_bias_pre_flt(curr_carr_biases, curr_carr_biases_num);
            if (fabs(carr_bias_cmp - carr_bias_pre_flt) < CARR_BIAS_OUT_THRESH || ++carr_bias_cmp_skip > CARR_BIAS_CMP_MAX_SKIP)
            {
                carr_bias_cmp = bias_smth->next(carr_bias_pre_flt);
                carr_bias_cmp_skip = 0;
            }
        }

        // comp mean carr bias dual
        if (rem_comp_bias && curr_carr_biases2_num)
        {
            carr_bias2_pre_ft = comp_carr_bias_pre_flt(curr_carr_biases2, curr_carr_biases2_num);
            if (fabs(carr_bias2_cmp - carr_bias2_pre_ft) < CARR_BIAS_OUT_THRESH || ++carr_bias2_cmp_skip > CARR_BIAS_CMP_MAX_SKIP)
            {
                carr_bias2_cmp = bias2_smth->next(carr_bias2_pre_ft);
                carr_bias2_cmp_skip = 0;
            }
        }

        if (fu_lag_startup)
        {
            fu_lag_startup--;
            continue;
        }

        if (!observables.read_binary_obs())
            break;

        std::map<int, Gnss_Synchro> gnss_synchro_map;

        double rx_time = 0;
        bool anyValid = false;

        bool error  = false;

        for (int n = 0; n < obs_n_channels; n++)
        {
            //std::cout << "prn=" << *true_obs_data.PRN << std::endl;
            //std::cout << "tow=" << *true_obs_data.TOW_at_current_symbol_s << std::endl;

            bool valid = static_cast<bool>(observables.valid[n]);

            //#warning dbg PRN OFF
            //if (observables.PRN[n] == 26)
            //    continue;

            if (!valid)
                continue;

            //if (observables.PRN[n] == 20)
            //    observables.Acc_carrier_phase_hz[n] = 0;

            //double ph_mul = trunc(observables.Acc_carrier_phase_hz[n] / 256 + .5);
            //observables.Acc_carrier_phase_hz[n] -= ph_mul * 256;
            //observables.Acc_carrier_phase_hz[n] /= 256;

            Gnss_Synchro gns_syn;

            // lock lost indicator
            bool lli = false;

            if (!isGalileo)
            {
                gns_syn.System = 'G';
                memcpy(gns_syn.Signal, "1C", 2);
            }
            else
            {
                gns_syn.System = 'E';
                memcpy(gns_syn.Signal, "1B", 2);
            }

            gns_syn.Flag_valid_word = valid;

            int prn = observables.PRN[n];
            int isDual = IS_DUAL(prn);

            #if 0
                #warning TEMP tylko pasmo 2
                if (isDual == 0)
                    continue;
            #endif

            if (isDual)
            {
                //#warning TEMP tylko pasmo 1
                //continue;

                prn -= 0x80;
                // 2S
                //gns_syn.Signal[0] = '2';
                //gns_syn.Signal[1] = 'S';

                if (!isGalileo)
                {
                    gns_syn.System = 'G';
                    memcpy(gns_syn.Signal, "L5", 2);
                }
                else
                {
                    gns_syn.System = 'E';
                    memcpy(gns_syn.Signal, "5X", 2);
                }

                //std::cout << "DUAL SEC OBS TEMP SKIP  " << std::endl;
                //continue;

            }
            gns_syn.PRN = prn;

#if 0
            gns_syn.RX_time = observables.RX_time[n];
            gns_syn.interp_TOW_ms = observables.TOW_at_current_symbol_s[n] * 1000;
            gns_syn.Carrier_Doppler_hz = observables.Carrier_Doppler_hz[n];
//#warning minus phase!
            gns_syn.Carrier_phase_rads = +observables.Acc_carrier_phase_hz[n] * TWO_PI;
            // std::cout << "ph = " << observables.Acc_carrier_phase_hz[n] << std::endl;
            gns_syn.Pseudorange_m = observables.Pseudorange_m[n];
#else
            gns_syn.RX_time = observables.RX_time[n];
            //gns_syn.RX_time = rx_smth[n]->next(observables.RX_time[n]);

            //gns_syn.RX_time = observables.RX_time[n] + 0.068;

            //gns_syn.RX_time = (int)(observables.RX_time[n]*1000) / 1000.0;
            //gns_syn.RX_time = (int)(observables.RX_time[n]*50) / 50.0;
            //printf("RX_time=%.15g\n", gns_syn.RX_time);

            //gns_syn.RX_time =
            //        (int)(observables.RX_time[n]*1000) / 1000.0 +
            //        observables.Pseudorange_m[n] / SPEESPEED_OF_LIGHT_M_S;

            //gns_syn.interp_TOW_ms = 0; //observables.TOW_at_current_symbol_s[n] * 1000;
            gns_syn.Carrier_Doppler_hz = 666; // observables.Carrier_Doppler_hz[n];
            gns_syn.CN0_dB_hz = 55;

            double carr = observables.Acc_carrier_phase_hz[n];

            if (0)
            {
                double range = observables.Pseudorange_m[n];
                double tm_dt = observables.RX_time[n] - prev_rxtime[n];
                if (prev_rxtime[n] >= 0.001 && tm_dt >= 0.001)
                {
                    int prn = observables.PRN[n];

                    double carr_f = -(carr - prev_carr[n]) / tm_dt;
                    double range_f = -(range - prev_range[n]) / tm_dt / LAMBDA_XX(IS_DUAL(prn));

                    if (fabs(carr_f - carr_bias0 - range_f) < REAL_BIAS_THRESH)
                    {
                        //carr_bias_cmp = bias_smth->next(carr_f - range_f);
                        if (!isDual)
                        {
                            curr_carr_biases[curr_carr_biases_num] = carr_f - range_f;
                            curr_carr_biases_num++;
                        }
                        else
                        {
                            curr_carr_biases2[curr_carr_biases2_num] = carr_f - range_f;
                            curr_carr_biases2_num++;
                        }
                    }
                    else
                    {
                        printf("*** freq bias comp ovf carr_fq = %g rg_fq=%g\n", carr_f, range_f);
                    }
                }
            }

            // wykrywanie dużego biasu range/carr
            if (1)
            {
                double range = observables.Pseudorange_m[n];
                double tm_dt = observables.RX_time[n] - prev_rxtime[n];
                if (prev_rxtime[n] >= 0.001 && tm_dt >= 0.001)
                {
                    int prn = observables.PRN[n];
                    double carr_f = -(carr - prev_carr[n]) / tm_dt;
                    double range_f = -(range - prev_range[n]) / tm_dt / LAMBDA_XX(IS_DUAL(prn));
                    if (fabs(carr_f - carr_bias0 - range_f) >= REAL_BIAS_THRESH)
                        lli = true;
                }
            }


            {

                //if (carr_rx0 < 0)
                //    carr_rx0 = observables.RX_time[n];

                if (prev_rxtime[n] == 0 || (prev_carr[n] != 0 && abs(prev_carr[n] - carr) > 100e6))
                {
                    carr_acc[n] = -carr;
                    // carr_acc[n] += (observables.RX_time[n] - carr_rx0) * carr_bias;
                    printf("-- %g rst carr sat %d\n", observables.RX_time[n], (int)observables.PRN[n]);
                    lli = true;
                }
                else
                {
                    double abias = !isDual ? carr_bias_cmp : carr_bias2_cmp;
                    carr_acc[n] += (observables.RX_time[n] - prev_rxtime[n]) * abias;
                }
            }

            prev_rxtime[n] = observables.RX_time[n];
            prev_carr[n] = carr;
            prev_range[n] = observables.Pseudorange_m[n];

            carr += carr_acc[n];

            //gns_syn.Carrier_phase_rads = observables.Acc_carrier_phase_hz[n] * GPS_TWO_PI;
            gns_syn.Carrier_phase_rads = carr * TWO_PI;
            //gns_syn.Carrier_phase_rads = carr_smth[n]->next(observables.Acc_carrier_phase_hz[n] * TWO_PI);

            //gns_syn.Carrier_phase_rads = -observables.Acc_carrier_phase_hz[n] * TWO_PI;
            //gns_syn.Carrier_phase_rads = ((int)observables.Acc_carrier_phase_hz[n]-1107) * GPS_TWO_PI;
            //gns_syn.Carrier_phase_rads = ((int)(observables.Acc_carrier_phase_hz[n] / 300)) * GPS_TWO_PI * 300.0;
            //gns_syn.Carrier_phase_rads = ((int)(-observables.Acc_carrier_phase_hz[n] / 1)) * TWO_PI * 1;
            //gns_syn.Carrier_phase_rads = 0;

            gns_syn.Pseudorange_m = observables.Pseudorange_m[n];
            gns_syn.interp_TOW_ms = gns_syn.RX_time - gns_syn.Pseudorange_m / SPEED_OF_LIGHT_M_S;

            //gns_syn.Pseudorange_m = rng_smth[n]->next(observables.Pseudorange_m[n]);

            //gns_syn.Pseudorange_m = +0.068 * SPEED_OF_LIGHT + observables.Pseudorange_m[n];
            //gns_syn.Pseudorange_m = (int)(observables.Pseudorange_m[n] * 1000) / 1000.0;

#endif

            // gdy CN0_dB_hz ==-1 to LLI
            if (lli)
            {
                gns_syn.CN0_dB_hz = -1;
                printf("\n*** LLI ***\n\n");
            }

            #if 0
            #warning dbg
            if (abs(gns_syn.RX_time - 557803) < 0.050)
                gns_syn.RX_time += 0.042;
            #endif


            if (0 && gns_syn.PRN == 1)
            {
                 std::cout << "*** dbg skip PRN " << std::endl;
                 continue;
            }

            //#warning TEMP DBG IF
            //if (gns_syn.PRN != 20)
            gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(n, gns_syn));
            //gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(gns_syn.PRN, gns_syn));

            //if (0 && epoch_counter > 4900 && epoch_counter < 4910)
            //if (1 && abs(gns_syn.RX_time - 518520.631) < 0.01)
            if (0 && abs(gns_syn.RX_time - 518490) < 2)
            {
                if (gns_syn.RX_time - prev_time > 1e-6)
                    std::cout << "***" << std::endl;

                std::cout << std::setprecision(12);
                std::cout << std::endl;
                std::cout << "  valid = " << valid << std::endl;
                std::cout << "    PRN = " << (int)gns_syn.PRN << std::endl;
                std::cout << "RX_time = " << gns_syn.RX_time << std::endl;
                std::cout << " TOW_ms = " << gns_syn.interp_TOW_ms << std::endl;
                std::cout << "p-range = " << gns_syn.Pseudorange_m << std::endl;
                if (n == obs_n_channels - 1)
                    std::cout << "---" << std::endl;
            }

            if (valid)
            {
                //#warning temp 0
                if (1 && anyValid && abs(rx_time - gns_syn.RX_time) > 1e-9)
                {
                    std::cout << std::endl << "*** ERROR : time change in one row/measure !!" << std::endl << std::endl;
                    error = true;
                    break;
                }

                rx_time = gns_syn.RX_time;

                if (gns_syn.RX_time - prev_time > 1e-6)
                {
                    time_epoch++;
                    prev_time = gns_syn.RX_time;
                }

                //if( first_rx_time < 0)
                //    first_rx_time = rx_time;

                anyValid = true;
            }

        } //for

        if (error)
        {
            // break;
            continue;
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

        //#warning dbg continue
        //if(1 && rx_time < 557850) //|| rx_time > 559700) // 567165) //566762)
        //    continue;

        if (last_eph_update_tm < 0 || rx_time - last_eph_update_tm > 30)
        {
            if (!isGalileo)
            {
                d_ls_pvt->gps_ephemeris_map = load_gps_ephemeris(eph_xml_filename, rx_time);
                if (d_ls_pvt->gps_ephemeris_map.size() == 0)
                {
                    std::cout << "NO EPHEMERIS FOUND!" << std::endl;
                    break;
                }
                last_eph_update_tm = rx_time;
            }
            else
            {
                d_ls_pvt->galileo_ephemeris_map = load_gal_ephemeris(eph_xml_filename, rx_time);
                if (d_ls_pvt->galileo_ephemeris_map.size() == 0)
                {
                    std::cout << "NO EPHEMERIS FOUND!" << std::endl;
                    //break; //początkowe TOW mogą być błędne
                }
                else
                    last_eph_update_tm = rx_time;
            }
        }

#if 0
#warning MOD TX TIME
        double max_rx = 0;
        for (auto it = gnss_synchro_map.cbegin(); it != gnss_synchro_map.cend(); it++)
            if(it->second.RX_time > max_rx) max_rx = it->second.RX_time;
        std::map<int, Gnss_Synchro> gnss_synchro_map2;
        for (auto it = gnss_synchro_map.cbegin(); it != gnss_synchro_map.cend(); it++)
        {
            Gnss_Synchro gns_syn = it->second;
            gns_syn.RX_time = max_rx;
            gnss_synchro_map2.insert(std::pair<int, Gnss_Synchro>(it->first, gns_syn));
        }
        gnss_synchro_map = gnss_synchro_map2;
#endif

        if (WRITE_OBS_CSV)
        {
            for (auto it = gnss_synchro_map.cbegin(); it != gnss_synchro_map.cend(); it++)
            {
                int n = it->first;
                write_obs_csv(fcsv_ch[n], &it->second, prev_csv_tm + n, prev_csv_carr + n, prev_csv_rg + n, bias_csv_smth + n);
            }
        }

        if (!isGalileo)
            check_gps_eph(gnss_synchro_map, d_ls_pvt->gps_ephemeris_map, rx_time);
        else
            check_gal_eph(gnss_synchro_map, d_ls_pvt->galileo_ephemeris_map, rx_time);

        if (!d_ls_pvt->get_PVT(gnss_synchro_map, false))
        {
            std::cout << "not comp" << std::endl;
            continue;
        }

        if (!d_ls_pvt->is_valid_position())
        {
            std::cout << "not valid" << std::endl;
            continue;
        }

        //d_ls_pvt->do_rtkoutstat();

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

        std::cout << "RX TOW = " << rx_time << std::endl;

        std::cout << "ECEF[m] = " << std::fixed << std::setprecision(16)
                  << d_ls_pvt->pvt_sol.rr[0] << ","
                  << d_ls_pvt->pvt_sol.rr[1] << ","
                  << d_ls_pvt->pvt_sol.rr[2] << std::endl;
        /* std::cout << "Dilution of Precision at " << boost::posix_time::to_simple_string(d_ls_pvt->get_position_UTC_time())
                 << " UTC using "<< d_ls_pvt->get_num_valid_observations() <<" observations is HDOP = " << d_ls_pvt->get_hdop() << " VDOP = "
                 << d_ls_pvt->get_vdop() */
        std::cout << "GDOP = " << d_ls_pvt->get_gdop() << std::endl;
        printf("num valid obs = %d \n", d_ls_pvt->get_num_valid_observations());

        double error_LLH_m = great_circle_distance(LLH(0), LLH(1), d_ls_pvt->get_latitude(), d_ls_pvt->get_longitude());
        std::cout << "2D error [m] : " << error_LLH_m << std::endl;

        arma::vec measured_r_eb_e = {d_ls_pvt->pvt_sol.rr[0], d_ls_pvt->pvt_sol.rr[1], d_ls_pvt->pvt_sol.rr[2]};

        arma::vec error_r_eb_e = measured_r_eb_e - true_r_eb_e;

        double error_3d_m = arma::norm(error_r_eb_e, 2);

        std::cout << "3D error [m] : " << error_3d_m << std::endl;

        if (error_3d_m >= error_bound) //200.0)
        {
            std::cout << "3D error BIG!" << std::endl;
            big_err_num++;
        }
        else
        {
            std::cout << "3D error OK!" << std::endl;
            sq_sum_ecef = sq_sum_ecef + arma::pow(error_r_eb_e, 2);
            sum_meas_pos_ecef = sum_meas_pos_ecef + measured_r_eb_e;
            sum_num++;
        }

        double max_abs_resp = 0;
        double max_abs_resc = 0;
        fprintf(resCsv, "%.12g", rx_time);
        for (int i = 0; i < MAXSAT; i++)
        {
            if (!d_ls_pvt->pvt_ssat[i].vs)
                continue;
            double resp = d_ls_pvt->pvt_ssat[i].resp[0];
            if (fabs(resp) > fabs(max_abs_resp))
                max_abs_resp = resp;
            double resc = d_ls_pvt->pvt_ssat[i].resc[0];
            if (fabs(resc) > fabs(max_abs_resc))
                max_abs_resc = resc;
            fprintf(resCsv, "; %g; %g", resp, resc);
        }
        fprintf(resCsv,"\n");

        fprintf(diffCsv, "%.12g; %g; %g; %g; %g; %g; %g; %g; %g\n", rx_time, error_3d_m, error_LLH_m, d_ls_pvt->get_gdop(), max_abs_resp, max_abs_resc, carr_bias_cmp - carr_bias0, carr_bias_pre_flt- carr_bias0, carr_bias2_cmp);

        if (save_gpx)
            gpx_dump.print_position(d_ls_pvt.get(), false);

    }


    std::cout << "-----------------" << std::endl;
    std::cout << "mode = " << positioning_mode_str << std::endl;
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

    std::cout << "DONE *" << std::endl;

    rtkclosestat();

    fclose(diffCsv);
    diffCsv = 0;

    fclose(resCsv);
    resCsv = 0;

    if (WRITE_OBS_CSV)
    {
        close_obs_csv(fcsv_ch, obs_n_channels);
    }

	return 0;
}

