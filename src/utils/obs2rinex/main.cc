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
#include "rinex_printer.h"


//#include "gnss_sdr_supl_client.h"
//#include "in_memory_configuration.h"
//#include "rtklib_solver.h"
//#include "rtklib_rtkpos.h"

#include "galileo_ephemeris.h"
#include "gps_ephemeris.h"
#include "gps_cnav_ephemeris.h"

#include "observables_dump_reader.h"
#include "armadillo"

#include "gnss_synchro.h"

//#include "MATH_CONSTANTS.h"

//#include "gpx_printer.h"

#include <stdio.h>
#include <assert.h>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/shared_ptr.hpp>
#include "file_configuration.h"


#define LOG(severity) std::cout

#define WRITE_OBS_CSV (0)

#define DUAL_RINEX (0)



void write_obs_csv(FILE *fcsv, const Gnss_Synchro *o, double *prev_tm, double *prev_carr)
{
    double ttime  = o->Pseudorange_m / SPEED_OF_LIGHT_M_S;// - .068;
    double carr = o->Carrier_phase_rads / TWO_PI;

    double tm_dt = o->RX_time - *prev_tm;
    double freq = 0;
    if(tm_dt >= 0.001)
        freq = (carr - *prev_carr) / (tm_dt);
    else
        tm_dt = 0.001;

    fprintf(
            fcsv,
            "%.15g; %.12g; %.20g; %d; %g; %.15g; %g\n",
            o->RX_time,
            o->Pseudorange_m,
            carr,
            o->PRN,
            o->Carrier_Doppler_hz,
            ttime,
            freq
    );
    *prev_carr = carr;
    *prev_tm = o->RX_time;
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
        fprintf(fcsv_ch[i], "rx_time; p-rng_ch%d; phase_cyc_ch%d; prn_ch%d; doppler; t-time; d/dt(carr)\n", i, i, i);

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

template <size_t N>
class MovingMean
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



int main(int argc, char** argv)
{

    if (argc < 4)
    {
        printf("args: obs-path channels week [isGalileo]\n");
        return 1;
    }

    //std::string conf_path(argv[1]);
    //std::cout << "conf_path = " << conf_path << std::endl;
    //std::shared_ptr<InMemoryConfiguration> configuration;
    //configuration = std::make_shared<InMemoryConfiguration>();
    //std::shared_ptr<FileConfiguration> configuration(new FileConfiguration(conf_path));
    //std::string obs_filename = configuration->property("obs_to_nav.obs_filename", std::string(""));
    //int obs_n_channels = configuration->property("obs_to_nav.obs_n_channels", 1);
    //double ref_ecef_X = configuration->property("obs_to_nav.ref_ecef_X", 0.0);
    //double ref_ecef_Y = configuration->property("obs_to_nav.ref_ecef_Y", 0.0);
    //double ref_ecef_Z = configuration->property("obs_to_nav.ref_ecef_Z", 0.0);
    //arma::vec true_r_eb_e = arma::vec({ ref_ecef_X, ref_ecef_Y, ref_ecef_Z });

    std::string obs_filename = std::string(argv[1]);
    int obs_n_channels = atoi(argv[2]);
    int week = atoi(argv[3]);

    int isGalileo = argc > 4 && atoi(argv[4]) != 0; //|| MK_MOD_GPS_AS_GALILEO;

    int decym = 1;

    Observables_Dump_Reader observables(obs_n_channels);  // 1 extra

    if (!observables.open_obs_file(obs_filename))
        std::cout << "Failure opening true observables file" << std::endl;

    if (!observables.read_binary_obs())
        std::cout << "Failure reading true tracking dump file." << std::endl;

    //arma::vec sq_sum_ecef = { 0.0, 0.0, 0.0};
    //arma::vec sum_meas_pos_ecef = { 0.0, 0.0, 0.0};

#if (0)
    arma::vec LLH =  LLH_to_deg(cart2geo(true_r_eb_e, 4));
#endif

    //std::size_t dirPos = obs_filename.find_last_of("/");
    //if (dirPos == std::string::npos)
    //    dirPos = 0;


    int64_t epoch_counter = 0;
    int time_epoch = 0;
    double prev_time = -1;


    FILE *fcsv_ch[obs_n_channels];
    double prev_csv_carr[obs_n_channels];
    double prev_csv_tm[obs_n_channels];

#if 0
    std::shared_ptr<MovingMean<50>> carr_smth[obs_n_channels];
    std::shared_ptr<MovingMean<50>> rng_smth[obs_n_channels];
    std::shared_ptr<MovingMean<50>> rx_smth[obs_n_channels];
    for (int i = 0; i < obs_n_channels; ++i)
    {
        carr_smth[i] = std::make_shared<MovingMean<50>>();
        rng_smth[i] = std::make_shared<MovingMean<50>>();
        rx_smth[i] = std::make_shared<MovingMean<50>>();
    }
#endif

    if (WRITE_OBS_CSV)
        start_obs_csv(obs_filename.c_str(), fcsv_ch, obs_n_channels);

    //int last_eph_update_tm = -1;

    std::shared_ptr<Rinex_Printer> rinex = std::make_shared<Rinex_Printer>();

    int rinex_hdr_wr = 0;

    Galileo_Ephemeris gal_eph = Galileo_Ephemeris();
    Gps_Ephemeris gps_eph = Gps_Ephemeris();
    Gps_CNAV_Ephemeris gps_cnav_eph = Gps_CNAV_Ephemeris();
    gps_cnav_eph.i_GPS_week = week;
    gal_eph.WN_5 = week;
    gps_eph.i_GPS_week = week;


    int prev_prn[obs_n_channels];
    double prn_start_tm[obs_n_channels];
    bool prev_lli[obs_n_channels];
    bool first_valid = true;

    for (int ii = 0; ii < obs_n_channels; ++ii)
    {
        prev_prn[ii] = -1;
        prev_lli[ii] = 0;
    }

    //int lli[obs_n_channels];

    observables.restart();

    while (observables.read_binary_obs())
    {
        std::map<int, Gnss_Synchro> gnss_synchro_map;

        double rx_time = 0;
        bool anyValid = false;
        bool error  = false;
        bool all_lli = false;

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

            Gnss_Synchro gns_syn;

            if (!isGalileo)
            {
                gns_syn.System = 'G';
                gns_syn.Signal[0] = '1';
                gns_syn.Signal[1] = 'C';
            }
            else
            {
                gns_syn.System = 'E';
                gns_syn.Signal[0] = '1';
                gns_syn.Signal[1] = 'B';
            }

            gns_syn.Flag_valid_word = valid;

            int prn = observables.PRN[n];

            #if 0
                #warning TEMP tylko pasmo 2
                if ((prn & 0x80) == 0)
                    continue;
            #endif

            if (prn & 0x80)
            {
                //#warning TEMP tylko pasmo 1
                //continue;

                prn -= 0x80;
                // 2S
                //gns_syn.Signal[0] = '2';
                //gns_syn.Signal[1] = 'S';

                if (!isGalileo)
                {
                    gns_syn.Signal[0] = 'L';
                    gns_syn.Signal[1] = '5';
                }
                else
                {
                    gns_syn.Signal[0] = '5';
                    gns_syn.Signal[1] = 'X';
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
#warning DBG MIN MEAS
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

            //gns_syn.Carrier_phase_rads = observables.Acc_carrier_phase_hz[n] * GPS_TWO_PI;
            gns_syn.Carrier_phase_rads = observables.Acc_carrier_phase_hz[n] * TWO_PI;
            //gns_syn.Carrier_phase_rads = carr_smth[n]->next(observables.Acc_carrier_phase_hz[n] * TWO_PI);

            //gns_syn.Carrier_phase_rads = -observables.Acc_carrier_phase_hz[n] * TWO_PI;
            //gns_syn.Carrier_phase_rads = ((int)observables.Acc_carrier_phase_hz[n]-1107) * GPS_TWO_PI;
            //gns_syn.Carrier_phase_rads = ((int)(observables.Acc_carrier_phase_hz[n] / 300)) * GPS_TWO_PI * 300.0;
            //gns_syn.Carrier_phase_rads = ((int)(-observables.Acc_carrier_phase_hz[n] / 1)) * TWO_PI * 1;
            //gns_syn.Carrier_phase_rads = 0;

            gns_syn.Pseudorange_m = observables.Pseudorange_m[n];
            gns_syn.interp_TOW_ms = gns_syn.RX_time - gns_syn.Pseudorange_m / SPEED_OF_LIGHT_M_S;

            //pseudorange_m = (self->rx_time - txTime) * c ;
            //pseudorange_m/c = self->rx_time - txTime;
            //txTime = self->rx_time - pseudorange_m/c;


            //gns_syn.Pseudorange_m = rng_smth[n]->next(observables.Pseudorange_m[n]);

            //gns_syn.Pseudorange_m = +0.068 * SPEED_OF_LIGHT + observables.Pseudorange_m[n];
            //gns_syn.Pseudorange_m = (int)(observables.Pseudorange_m[n] * 1000) / 1000.0;

#endif

            // udawanie LLI po dodaniu nowej sat
            if (1 && valid)
            {
                if (first_valid)
                    prn_start_tm[n] = observables.RX_time[n] - 1000;
                else if (prev_prn[n] != prn) // prev_prn[n] > 0 &&
                    prn_start_tm[n] = observables.RX_time[n];

                prev_prn[n] = prn;

                //!first_valid &&
#warning warunek z góry długie blokowanie sat, trzeba zrobić inteligentniej
                //TODO eksperymenty ze zmniejszeniem tego czasu
                bool lli = observables.RX_time[n] - prn_start_tm[n] < 20;

                if (prev_lli[n] && !lli)
                    all_lli = true;
                prev_lli[n] = lli;

                if (lli)
                    gns_syn.CN0_dB_hz = -1;
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

        }

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

        first_valid = false;

        if (time_epoch % decym)
            continue;



        // if (epoch_counter < 20)
        //    continue;

        //#warning dbg continue
        //if(1 && rx_time < 557850) //|| rx_time > 559700) // 567165) //566762)
        //    continue;

        if (all_lli)
        {
            std::map<int, Gnss_Synchro> gnss_synchro_map_new;
            for (auto it = gnss_synchro_map.cbegin(); it != gnss_synchro_map.cend(); it++)
            {
                Gnss_Synchro gns_syn = it->second;
                gns_syn.CN0_dB_hz = -1;
                gnss_synchro_map_new.insert(std::pair<int, Gnss_Synchro>(it->first, gns_syn));
            }
            gnss_synchro_map = gnss_synchro_map_new;
        }

        if (WRITE_OBS_CSV)
        {
            for (auto it = gnss_synchro_map.cbegin(); it != gnss_synchro_map.cend(); it++)
                write_obs_csv(fcsv_ch[it->first], &it->second, prev_csv_tm + it->first, prev_csv_carr + it->first);
        }

        if (!rinex_hdr_wr)
        {
            if (!isGalileo)
            {
                //assert(first_rx_time > 0);
                rinex_hdr_wr = 1;
                if (!DUAL_RINEX)
                    rinex->rinex_obs_header(rinex->obsFile, gps_eph, rx_time);
                else
                    rinex->rinex_obs_header(rinex->obsFile, gps_eph, gps_cnav_eph, rx_time, "1C L5");

            }
            else
                rinex->rinex_obs_header(rinex->obsFile, gal_eph, rx_time);
        }

        if (!isGalileo)
        {
            //rinex_eph_gps_cnav
            if (!DUAL_RINEX)
                rinex->log_rinex_obs(rinex->obsFile, gps_eph, rx_time, gnss_synchro_map);
            else
                rinex->log_rinex_obs(rinex->obsFile, gps_eph, gps_cnav_eph, rx_time, gnss_synchro_map);

        }
        else
            rinex->log_rinex_obs(rinex->obsFile, gal_eph, rx_time, gnss_synchro_map);


        std::streamsize ss = std::cout.precision();  // save current precision
        std::cout.setf(std::ios::fixed, std::ios::floatfield);

        auto facet = new boost::posix_time::time_facet("%Y-%b-%d %H:%M:%S.%f %z");
        std::cout.imbue(std::locale(std::cout.getloc(), facet));

        std::cout << std::setprecision(ss);

        std::cout << "RX TOW = " << rx_time << std::endl;


    }


    std::cout << "-----------------" << std::endl;
    std::cout << "epoch_counter = " << epoch_counter << std::endl;
    std::cout << "time_epoch    = " << time_epoch << std::endl;
    std::cout << "rinex    : " << rinex->obsfilename.c_str() << std::endl;
    std::cout << "DONE" << std::endl;


    if (WRITE_OBS_CSV)
    {
        close_obs_csv(fcsv_ch, obs_n_channels);
    }

	return 0;
}

