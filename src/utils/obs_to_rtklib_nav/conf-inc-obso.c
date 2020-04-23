/*
 * conf-inc-obso.c
 *
 *  Created on: Apr 23, 2020
 *      Author: mk
 */




    // TODO file sel

    //std::string obs_filename = std::string("/home/mk/Gnss/gnss-sdr-my/observables.dat");

    // opcja filtrowania efemeryd wg TOE, -1 oznacza brak filtrowania
    //int find_toe = -1;

#if 0
    const int error_bound = 200;

    // gps-sdr-sim ref pos
    arma::vec true_r_eb_e = arma::vec({ 3655463.659, 1404112.314, 5017924.853 });

    int obs_n_channels = 5;
    std::string obs_filename = std::string("/home/mk/Gnss/Results/2020-04-05/1/observables.dat");
    //std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-05/1/gps_ephemeris.xml";
    std::string eph_xml_filename = "/home/mk/Gnss/gnss-sdr-my/gps_ephemeris_full_brdc3540_14n.xml";
    //find_toe = 518400;

    std::string iono_xml_filename = "";

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

    int obs_n_channels = 5;
    std::string obs_filename = "/home/mk/Gnss/Results/2019-09-07/2/ticksExSavIQ.bin_obs.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-05/1/gps_ephemeris.xml";

#endif

#if 0
    // Single, iono+tropo OFF, mean - ref [m] = 7.04, 3D SD[m] = 18.36 ???
    // arma::vec true_r_eb_e = { 3655479.590, 1404123.428, 5017946.443};
    // 3D SD[m] = 92m

    int obs_n_channels = 8;
    std::string obs_filename = "/home/mk/Gnss/Results/2020-04-07/1/OnlineScanNav-gnss-sim-observables-ok.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-07/1/gps_ephemeris.xml";
#endif

#if 0
    // PPP_Static, elev 0, iono off, sd-3d[m] = 0.0005-NIEPRAWDA
    // ref = { 3655482.807, 1404118.578, 5017917.000};
    int obs_n_channels = 8;
    std::string obs_filename = "/home/mk/Gnss/Results/2020-04-07/2-OnlineScanNav/observables.dat";
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

    int obs_n_channels = 8;
    std::string obs_filename = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-1/observables.dat";
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
    int obs_n_channels = 8;
    std::string obs_filename = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-2/observables.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-2/gps_ephemeris.xml";
#endif

#if 0
    //16MHz, PPP, 3D SD[m] = 47.00
    const int error_bound = 200;
    arma::vec true_r_eb_e = { 3655432.602, 1404129.254, 5017900.940};

    int obs_n_channels = 8;
    std::string obs_filename = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-3/observables.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-3/gps_ephemeris.xml";
#endif

#if 0
    //16MHz, PPP, 3D SD[m] = 49.00
    const int error_bound = 200;
    arma::vec true_r_eb_e = { 3655432.872, 1404130.188, 5017901.024};
    //16MHz, PPP, 3D SD[m] = 27.61
    //const int error_bound = 40;
    //arma::vec true_r_eb_e = { 3655439.9541424359194934, 1404114.2663816625718027, 5017912.8526892913505435};


    int obs_n_channels = 8;
    std::string obs_filename = "/home/mk/Gnss/Results/2020-04-08/OnlineScanNav-4/observables.dat";
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

    int obs_n_channels = 8;
    std::string obs_filename = "/home/mk/Gnss/Results/2020-04-09/OnlineScanNav-1/observables.dat";
    std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-09/OnlineScanNav-1/gps_ephemeris.xml";
#endif


#if 0
    arma::vec true_r_eb_e;

    //const int error_bound = 200;
    const int error_bound = 20;

    // gps-sdr-sim ref pos
    true_r_eb_e = arma::vec({ 3655463.659, 1404112.314, 5017924.853 });

    int obs_n_channels = 8;
    //std::string obs_filename = "/home/mk/Gnss/Results/2020-04-09/OnlineScanNav-3-plus/observables.dat";
    //std::string obs_filename = "/home/mk/Gnss/Results/2020-04-09/OnlineScanNav-4-zero/observables.dat";
    std::string obs_filename = "/home/mk/Gnss/Results/2020-04-11/3-ok/observables.q32.dat";

    //std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-09/OnlineScanNav-3-plus/gps_ephemeris.xml";
    std::string eph_xml_filename = "/home/mk/Gnss/gnss-sdr-my/gps_ephemeris_full_brdc3540_14n.xml";

    std::string iono_xml_filename = "";

    //find_toe = 518400;

#endif


#if 0
    // current comp
    arma::vec true_r_eb_e;

    const int error_bound = 50;
    //const int error_bound = 10;

    // gps-sdr-sim ref pos
    true_r_eb_e = arma::vec({ 3655463.659, 1404112.314, 5017924.853 });

    //Kobyłka ref v2 2019-06-03
    //true_r_eb_e = arma::vec({3642332.408, 1411096.212, 5025380.626});

    //std::string dir = "/home/mk/Gnss/Results/2020-04-21/1/";
    std::string dir = "/home/mk/Gnss/SimpRel/";

    int obs_n_channels = 8;
    //std::string obs_filename = "/home/mk/Gnss/NaviSocRepo/tests/OnlineScanNav/observables.dat";
    //std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-05/1/gps_ephemeris.xml";
    //std::string eph_xml_filename = "/home/mk/Gnss/gnss-sdr-my/gps_ephemeris_full_brdc3540_14n.xml";
    std::string obs_filename = dir + "observables.q32.dat";
    std::string eph_xml_filename = dir + "gps_ephemeris.xml";
    std::string iono_xml_filename = dir + "gps_iono.xml";

    //find_toe = 518400;
    //find_toe = 547200;
    //find_toe = 554400;
    //find_toe = 561600;
    //find_toe = 568800;


#endif


    // load ephemeris
    //std::string eph_xml_filename = path + "data/rtklib_test/eph_GPS_L1CA_test1_poprawiony.xml";
    // eph pochodzą z uruchomienia gnss-sdr na pliku z gns-sdr-sim
    //std::string eph_xml_filename = "data/rtklib_test/eeph_gpssim_pw.xml";
    //std::string eph_xml_filename = "/home/mk/Gnss/gnss-sdr-my/gps_ephemeris.xml";
    //std::string eph_xml_filename = "/home/mk/Gnss/Results/2020-04-05/1/gps_ephemeris.xml";

