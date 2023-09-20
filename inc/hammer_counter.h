#ifndef __HAMMERCOUNTER_H
#define __HAMMERCOUNTER_H
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <math.h>
#include <random>
#include <string>
#include <vector>

#include "champsim_constants.h"
#include <bits/stdc++.h>

#define RH_READ 0
#define RH_WRITE 1
#define RH_REFRESH 2
#define CPU_FREQ 4000
class Address
{
public:
  // channel
  // bank
  // rank
  // row
  std::tuple<uint64_t, uint64_t, uint64_t, uint64_t> addr;

  Address(uint64_t channel, uint64_t bank, uint64_t rank, uint64_t row) { addr = std::tuple<uint64_t, uint64_t, uint64_t, uint64_t>(channel, bank, rank, row); }

  uint64_t get_row() const { return (std::get<3>(addr)); }

  uint64_t get_rank() const { return (std::get<2>(addr)); }

  uint64_t get_bank() const { return (std::get<1>(addr)); }

  uint64_t get_channel() const { return (std::get<0>(addr)); }

  bool operator<(const Address& a) const { return (addr < a.addr); }
};

struct Count {
  uint64_t lost_hammers_to_refresh;
  uint64_t lost_hammers_to_access;

  uint64_t total_hammers;
  uint64_t normal_hammers;
  uint64_t prefetch_hammers;

  uint64_t highest_hammer_count;
  uint64_t highest_prefetch_hammer_count;
  uint64_t highest_normal_hammer_count;

  uint64_t start_cycle;
  bool truncated_by_refresh = false;
  bool is_refresh_only = true;
  Count();
  bool operator<(Count& b);
};

class HammerCounter
{
  std::map<Address, Count> row_open_master;
  std::map<Address, Count> row_open_counter;
  std::map<uint64_t, uint64_t> read_access_histogram;
  std::map<uint64_t, uint64_t> write_access_histogram;

  static std::string output_f;

  static uint64_t target_row;
  static uint64_t target_cycle;

  uint64_t phase;

  // cycle values
  uint64_t highest_hammers_per_cycle;
  uint64_t highest_hammers_per_cycle_p;
  uint64_t highest_hammer_row;
  uint64_t last_hammer_cycles;

  // calculated values
  uint64_t rows_per_refresh;
  uint64_t cycles_per_refresh;
  uint64_t cycles_per_bin;
  uint64_t cycles_per_heartbeat;

  // cumulative stats
  uint64_t row_charges_r;
  uint64_t row_charges_rp;
  uint64_t row_charges_rn;
  uint64_t row_charges_w;
  uint64_t row_charges_wp;
  uint64_t row_charges_wn;
  uint64_t refreshes;
  uint64_t refresh_row;
  uint64_t refresh_cycles;
  uint64_t total_cycles;
  uint64_t row_charges_ref;

  uint64_t unique_rows_visited;
  uint64_t lost_hammers;
  uint64_t lost_hammers_to_access;
  uint64_t lost_hammers_to_refresh;
  uint64_t victim_reads;
  uint64_t victim_writes;

  uint64_t channel_num;

  // utility funcs
  void flush();
  void InsertIntoMap(Address A, Count C);

public:
  HammerCounter();
  //~HammerCounter();
  static void set_output_file(std::string f);
  static std::string get_output_file();
  void log_charge(Address addr, int type, bool prefetch,uint64_t cycle);
  void log_write(Address addr);
  void log_cycle();
  //void log_refresh(uint64_t row, uint64_t cycle);
  static void set_target_row(uint64_t row);
  bool is_start_cycle(uint64_t cycle);

  static uint64_t get_target_row();
  static void set_target_cycle(uint64_t cycle);
  void print_file();
};

#endif
