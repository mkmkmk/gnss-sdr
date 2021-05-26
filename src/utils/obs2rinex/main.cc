/*!
 * \file main.cc
 * \brief obs_to_rtklib clone, accepts observables.dat, rinex generator
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

//#define DUAL_RINEX (0)

#define LAMBDA_L1 ( SPEED_OF_LIGHT_M_S / 1540 / 1023000)
#define LAMBDA_L5 ( SPEED_OF_LIGHT_M_S / 1150 / 1023000)
#define LAMBDA_XX(IS_DUAL) ((IS_DUAL) ? LAMBDA_L5 : LAMBDA_L1)

#define IS_DUAL(ARG) ((ARG) & 0x80)

// -- ad usuwanie odstających biasu częstotliwości
// liczba kolejnych odstających od ogólnej średniej po której kolejne przestają być odstającymi
#define CARR_BIAS_CMP_MAX_SKIP (5)

// próg realnego biasu
#define REAL_BIAS_THRESH (5000)

// próg w Hz na odstających z ogólnej średniej biasu
#define CARR_BIAS_OUT_THRESH (100)

// długość uśredniania średniego biasu
#define BIAS_SMOOTH_MEAN_N (250)


void write_obs_csv(FILE *fcsv, const Gnss_Synchro *o, double *prev_tm, double *prev_carr, double *prev_rg)
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
        carr_f = (carr - *prev_carr) / tm_dt;
        range_f = -(o->Pseudorange_m - *prev_rg) / tm_dt / LAMBDA_XX(isDual);

        if (fabs(carr_f - range_f) > 10)
                printf("*** SAT %d BIG freq bias = %g, carr_fq = %g, rg_fq=%g \n", o->PRN, carr_f-range_f, carr_f, range_f);

        if (fabs(carr_f-range_f) < 5000)
            bias_f = carr_f - range_f;
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


double truncmod(double a, double b)
{
    return a - b * trunc(a / b);
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


int rd_bias_csv_next(FILE* fcsv, double *tow, double *bias_band1, double *bias_band2)
{
    char *token, *saveptr, *dline, *endptr;
    char line[1024];

    *tow = 0;
    *bias_band1 = 0;
    *bias_band2 = 0;
    double val;

    if (!fgets(line, 1024, fcsv))
        return 0;

    dline = line;

    for (int i = 0; 3; ++i)
    {
        token = strtok_r(dline, ";\n", &saveptr);
        dline = NULL;
        if (!token)
            break;
        val = strtod(token, &endptr);

        if (endptr == token)
            printf("\nparse error?? ('%s')\n", token);

        if (i == 0)
            *tow = val;
        else if (i == 1)
            *bias_band1 = val;
        else if (i == 2)
            *bias_band2 = val;
    }

    return 1;
}


int main(int argc, char** argv)
{

    if (argc < 4)
    {
        printf("args: obs-path channels week [isGalileo] [isDual] [carrBias] \n");
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

    int isDual = 0;
    if (argc > 5)
        isDual = atoi(argv[5]);

    //double carr_bias = 0;
    double carr_bias0 = 0;
    if (argc > 6)
        carr_bias0 = atof(argv[6]);


    printf("obs_n_channels = %d\n", obs_n_channels);
    printf("week           = %d\n", week);
    printf("isGalileo      = %d\n", isGalileo);
    printf("isDual         = %d\n", isDual);
    printf("carr_bias0      = %.10g\n", carr_bias0);

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
    double prev_csv_carr[obs_n_channels] = {0};
    double prev_csv_rg[obs_n_channels] = {0};
    double prev_csv_tm[obs_n_channels] = {0};

#if 0
    std::shared_ptr<MovingAv<50>> carr_smth[obs_n_channels];
    std::shared_ptr<MovingAv<50>> rng_smth[obs_n_channels];
    std::shared_ptr<MovingAv<50>> rx_smth[obs_n_channels];
    for (int i = 0; i < obs_n_channels; ++i)
    {
        carr_smth[i] = std::make_shared<MovingAv<50>>();
        rng_smth[i] = std::make_shared<MovingAv<50>>();
        rx_smth[i] = std::make_shared<MovingAv<50>>();
    }
#endif

    if (WRITE_OBS_CSV)
        start_obs_csv(obs_filename.c_str(), fcsv_ch, obs_n_channels);

    FILE *pBiasCsv = fopen((obs_filename + "_preBias.csv").c_str(), "w");
    fprintf(pBiasCsv, "time;bias-band1;bias-band2\n");

    std::string rdBiasPath = obs_filename + "_preBias.csv-ft.csv";
    FILE *rdBiasCsv = fopen(rdBiasPath.c_str(), "r");
    char line[1024];

    if (rdBiasCsv)
    {
        if (fgets(line, 1024, rdBiasCsv))
            printf("*** reading filtered biases file\n");
        else
        {
            fclose(rdBiasCsv);
            rdBiasCsv = 0;
        }
    }
    if (!rdBiasCsv)
        printf("*** filtered biases file not found or open error (%s)\n", rdBiasPath.c_str());

    //int last_eph_update_tm = -1;

    std::shared_ptr<Rinex_Printer> rinex = std::make_shared<Rinex_Printer>();

    int rinex_hdr_wr = 0;

    Galileo_Ephemeris gal_eph = Galileo_Ephemeris();
    Gps_Ephemeris gps_eph = Gps_Ephemeris();
    Gps_CNAV_Ephemeris gps_cnav_eph = Gps_CNAV_Ephemeris();
    gps_cnav_eph.i_GPS_week = week;
    gal_eph.WN_5 = week;
    gps_eph.i_GPS_week = week;


    bool first_valid = true;
    int prev_prn[obs_n_channels] = {0};
    double prev_carr_rad[obs_n_channels] = {0};
    double prev_carr[obs_n_channels] = {0};
    double prev_rxtime[obs_n_channels] = {0};
    double carr_acc[obs_n_channels] = {0};
    double prev_range[obs_n_channels] = {0};

    for (int ii = 0; ii < obs_n_channels; ++ii)
        prev_prn[ii] = -1;

    double carr_bias_cmp = carr_bias0;
    double carr_bias2_cmp = carr_bias0;
    double carr_bias_pre_flt = 0;
    double carr_bias2_pre_ft = 0;

    int carr_bias_cmp_skip = 0;
    int carr_bias2_cmp_skip = 0;

    auto bias_smth = std::make_shared<MovingAv<BIAS_SMOOTH_MEAN_N>>();
    auto bias2_smth = std::make_shared<MovingAv<BIAS_SMOOTH_MEAN_N>>();


    //int lli[obs_n_channels];

    observables.restart();

    while (observables.read_binary_obs())
    {
        double curr_carr_biases[obs_n_channels];
        int curr_carr_biases_num = 0;

        double curr_carr_biases2[obs_n_channels];
        int curr_carr_biases2_num = 0;

        double lastobstm = 0;

        for (int n = 0; n < obs_n_channels; n++)
        {
            bool valid = static_cast<bool>(observables.valid[n]);
            if (!valid)
                continue;

            int prn = observables.PRN[n];
            lastobstm = observables.RX_time[n];

            //if (IS_DUAL(prn))
            //    continue;

            double carr = observables.Acc_carrier_phase_hz[n];
            double range = observables.Pseudorange_m[n];
            double tm_dt = observables.RX_time[n] - prev_rxtime[n];
            if (prev_rxtime[n] >= 0.001 && tm_dt >= 0.001)
            {
                double carr_f = -(carr - prev_carr[n]) / tm_dt;
                double range_f = -(range - prev_range[n]) / tm_dt / LAMBDA_XX(IS_DUAL(prn));

                //double carr_f = -(carr - prev_carr[n]) ;
                //double range_f = -(range - prev_range[n]) / LAMBDA_XX(IS_DUAL(prn));

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
            //prev_rxtime[n] = observables.RX_time[n];
            //prev_carr[n] = carr;
            //prev_range[n] = range;
        }

        // comp mean carr bias
        if (/*rem_comp_bias &&*/ curr_carr_biases_num)
        {
            carr_bias_pre_flt = comp_carr_bias_pre_flt(curr_carr_biases, curr_carr_biases_num);
            if (fabs(carr_bias_cmp - carr_bias_pre_flt) < CARR_BIAS_OUT_THRESH || ++carr_bias_cmp_skip > CARR_BIAS_CMP_MAX_SKIP)
            {
                carr_bias_cmp = bias_smth->next(carr_bias_pre_flt);
                carr_bias_cmp_skip = 0;
            }
        }

        fprintf(pBiasCsv, "%.12g;%.12g;%.12g\n", lastobstm, carr_bias_pre_flt, carr_bias2_pre_ft);

        // comp mean carr bias dual
        if (/*rem_comp_bias &&*/ curr_carr_biases2_num)
        {
            carr_bias2_pre_ft = comp_carr_bias_pre_flt(curr_carr_biases2, curr_carr_biases2_num);
            if (fabs(carr_bias2_cmp - carr_bias2_pre_ft) < CARR_BIAS_OUT_THRESH || ++carr_bias2_cmp_skip > CARR_BIAS_CMP_MAX_SKIP)
            {
                carr_bias2_cmp = bias2_smth->next(carr_bias2_pre_ft);
                carr_bias2_cmp_skip = 0;
            }
        }

        if (rdBiasCsv)
        {
            double rd_tow, rd_bias_band1, rd_bias_band2;
            int rd_bias_ok = rd_bias_csv_next(rdBiasCsv, &rd_tow, &rd_bias_band1, &rd_bias_band2);

            if (rd_bias_ok)
                rd_bias_ok = fabs(rd_tow - lastobstm) < 1e-3;

            if (rd_bias_ok)
            {
                carr_bias_cmp = rd_bias_band1;
                carr_bias2_cmp = rd_bias_band2;
                printf("readed filtered bias\n");
            }
            else
            {
                printf("missed filtered bias\n");
            }
        }

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

            if (fabs(carr - -125937220.116) <= 0.001)
            {
                printf("dbg\n");
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

            //if (carr_bias != 0)
            {

                if (prev_rxtime[n] == 0 || (prev_carr[n] != 0 && abs(prev_carr[n] - carr) > 100e6))
                {
                    carr_acc[n] = -carr;
                    printf("-- %g rst carr sat %d\n", observables.RX_time[n], (int)observables.PRN[n]);
                    lli = true;
                }
                else
                {
                    double abias = !isDual ? carr_bias_cmp : carr_bias2_cmp;
                    carr_acc[n] += (observables.RX_time[n] - prev_rxtime[n]) * abias;
                }
            }

            prev_carr[n] = carr;

            carr += carr_acc[n];

            //gns_syn.Carrier_phase_rads = observables.Acc_carrier_phase_hz[n] * GPS_TWO_PI;
            gns_syn.Carrier_phase_rads = carr * TWO_PI;

            // 1e9 rollover bo nie mieści się w rinex
            gns_syn.Carrier_phase_rads = truncmod(gns_syn.Carrier_phase_rads, 1e9 * TWO_PI);

            //gns_syn.Carrier_phase_rads = carr_smth[n]->next(observables.Acc_carrier_phase_hz[n] * TWO_PI);

            //gns_syn.Carrier_phase_rads = -observables.Acc_carrier_phase_hz[n] * TWO_PI;
            //gns_syn.Carrier_phase_rads = ((int)observables.Acc_carrier_phase_hz[n]-1107) * GPS_TWO_PI;
            //gns_syn.Carrier_phase_rads = ((int)(observables.Acc_carrier_phase_hz[n] / 300)) * GPS_TWO_PI * 300.0;
            //gns_syn.Carrier_phase_rads = ((int)(-observables.Acc_carrier_phase_hz[n] / 1)) * TWO_PI * 1;
            //gns_syn.Carrier_phase_rads = 0;

            gns_syn.Pseudorange_m = observables.Pseudorange_m[n];
            gns_syn.interp_TOW_ms = gns_syn.RX_time - gns_syn.Pseudorange_m / SPEED_OF_LIGHT_M_S;

            // wykrywanie skoków tx time ("*** rx_time_set ***")
            // - niezbyt dobrze działa
            if (observables.RX_time[n] - prev_rxtime[n] < 1e-3)
            {
                lli = true;
            }
            else if (fabs(gns_syn.Pseudorange_m - prev_range[n]) / (observables.RX_time[n] - prev_rxtime[n]) > 10000)
            {
                lli = true;
            }

            prev_range[n] = gns_syn.Pseudorange_m;
            prev_rxtime[n] = observables.RX_time[n];

            //pseudorange_m = (self->rx_time - txTime) * c ;
            //pseudorange_m/c = self->rx_time - txTime;
            //txTime = self->rx_time - pseudorange_m/c;

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

            // flagi LLI po dodaniu nowej sat lub carr rollover
            if (1 && valid)
            {
                if (!first_valid && prev_prn[n] != prn)
                    all_lli = true;
                prev_prn[n] = prn;

                // LLI na całej epoce gdy następuje rollover fazy
                if (fabs(prev_carr_rad[n] - gns_syn.Carrier_phase_rads) > 1e8 * TWO_PI)
                    all_lli = true;

                //prev_carr[n] =  gns_syn.Carrier_phase_rads;
            }

            prev_carr_rad[n] = gns_syn.Carrier_phase_rads;

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

        first_valid = false;

        if (decym > 1 && (time_epoch % decym))
            continue;

        // if (epoch_counter < 20)
        //    continue;

        //#warning dbg continue
        //if(1 && rx_time < 557850) //|| rx_time > 559700) // 567165) //566762)
        //    continue;

        if (all_lli)
        {
            std::cout << "  all LLI " << std::endl;
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
            {
                int n = it->first;
                write_obs_csv(fcsv_ch[n], &it->second, prev_csv_tm + n, prev_csv_carr + n, prev_csv_rg + n);
            }
        }

        if (!rinex_hdr_wr)
        {
            if (!isGalileo)
            {
                //assert(first_rx_time > 0);
                rinex_hdr_wr = 1;
                if (!isDual)
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
            if (!isDual)
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

    std::ifstream  src(rinex->obsfilename, std::ios::binary);
    std::ofstream  dst(obs_filename+".O",   std::ios::binary);
    dst << src.rdbuf();

    if (rdBiasCsv)
        fclose(rdBiasCsv);
    fclose(pBiasCsv);

    if (WRITE_OBS_CSV)
    {
        close_obs_csv(fcsv_ch, obs_n_channels);
    }

    return 0;
}

