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

#ifdef RAMULATOR
#include "base/base.h"
#include "../ramulator2/src/dram_controller/controller.h"
#include "../ramulator2/src/dram_controller/plugin.h"

#endif


#define RH_READ 0
#define RH_WRITE 1
#define RH_REFRESH 2
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
struct HighestCount
{
  uint64_t hammer_count;
  uint64_t prefetch_hammer_count;
  uint64_t write_back_count;
  uint64_t normal_hammer_count;
  bool truncated_by_refresh;
  uint64_t start_cycle;
  bool hammered_besides_refresh;
  HighestCount();
};
struct LifetimeCount
{
  uint64_t lost_hammers_to_refresh;
  uint64_t lost_hammers_to_access;

  uint64_t total_hammers;
  uint64_t normal_hammers;
  uint64_t prefetch_hammers;
  uint64_t write_back_hammers;

  bool is_refresh_only;
  LifetimeCount();
};
struct Count {
  LifetimeCount lifetime;
  HighestCount highest;

  Count();
  bool operator<(Count& b);

  void CopyInto(Count C);
  void Combine(Count C);
};

class HammerCounter
{
  uint64_t dram_rows;
  uint64_t dram_ranks;
  uint64_t dram_banks;
  uint64_t dram_columns;
  uint64_t dram_channels;
  uint64_t dram_cap;



  std::map<Address, Count> row_open_master;
  std::map<Address, Count> row_open_counter;
  std::map<uint64_t, uint64_t> read_access_histogram;
  std::map<uint64_t, uint64_t> write_access_histogram;
  std::map<uint64_t, uint64_t> pref_access_histogram;
  std::map<uint64_t, uint64_t> wb_access_histogram;

  static std::string output_f;

  static uint64_t target_row;
  static uint64_t target_cycle;

  uint64_t phase;

  // cycle values
  uint64_t highest_hammers_per_cycle;
  uint64_t highest_hammers_per_cycle_p;
  uint64_t highest_hammers_per_cycle_wb;
  uint64_t highest_hammer_row;
  uint64_t last_hammer_cycles;
  

  // cumulative stats
  uint64_t row_charges_r;
  uint64_t row_charges_rp;
  uint64_t row_charges_rn;
  uint64_t row_charges_w;
  uint64_t row_charges_wb;
  uint64_t row_charges_wp;
  uint64_t row_charges_wn;
  uint64_t refreshes;
  uint64_t refresh_row;
  uint64_t refresh_cycles;
  uint64_t row_charges_ref;

  uint64_t lost_hammers;
  uint64_t lost_hammers_to_access;
  uint64_t lost_hammers_to_refresh;
  uint64_t victim_reads;
  uint64_t victim_writes;

  

  // utility funcs
  void flush();
  void InsertIntoMap(Address A, Count C);

public:
  static uint64_t processed_packets;
  // calculated values
  uint64_t cycles_per_bin;
uint64_t cycles_per_heartbeat;
uint64_t channel_num;
  uint64_t total_cycles;
  HammerCounter();
  //~HammerCounter();
  static void set_output_file(std::string f);
  static std::string get_output_file();
  void log_charge(Address addr,uint64_t p_addr, uint64_t v_addr, int type, bool prefetch,uint64_t cycle, bool write_back);
  void log_refresh(Address addr);
  void log_write(Address addr);
  void log_cycle();
  //void log_refresh(uint64_t row, uint64_t cycle);
  static void set_target_row(uint64_t row);
  bool is_start_cycle(uint64_t cycle);

  void set_cycles_per_bin(uint64_t c_p_b)   {cycles_per_bin = c_p_b; cycles_per_heartbeat = cycles_per_bin * 5;}; // 500us heartbeat
  void set_dram_rows(uint64_t row_count)    {dram_rows = row_count;};
  void set_dram_ranks(uint64_t rank_count)  {dram_ranks = rank_count;};
  void set_dram_banks(uint64_t bank_count)  {dram_banks = bank_count;};
  void set_dram_columns(uint64_t column_count) {dram_columns = column_count;};
  void set_dram_channels(uint64_t channel_count) {dram_channels = channel_count;};
  void set_dram_cap(uint64_t dram_count) {dram_cap = dram_count;};
  static uint64_t get_target_row();
  static void set_target_cycle(uint64_t cycle);
  void print_file();
};

#ifdef RAMULATOR
namespace Ramulator
{
class HammerCounterPlugin : public IControllerPlugin, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, HammerCounterPlugin, "HammerCounter", "Counts Hammer Activity for Research.")

  uint64_t last_bank_util = 0;
  double rb_miss = 0;
  double rb_hits = 0;
  double last_rb_miss = 0;
  double last_rb_hits = 0;
  private:
    IDRAM* m_dram = nullptr;
    IDRAMController* m_controller = nullptr;
    IMemorySystem* m_system = nullptr;
    HammerCounter HC;


  public:
    void init() override
    {

    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_ctrl = cast_parent<IDRAMController>();
      m_dram = m_ctrl->m_dram;
      m_controller = m_ctrl;
      m_system = memory_system;

      HC.set_dram_channels(m_dram->get_level_size("channel"));
      HC.set_dram_ranks(m_dram->get_level_size("rank"));
      HC.set_dram_banks(m_dram->get_level_size("bank") * m_dram->get_level_size("bankgroup"));
      HC.set_dram_columns(m_dram->get_level_size("column"));
      HC.set_dram_rows(m_dram->get_level_size("row"));

      HC.set_cycles_per_bin(uint64_t((uint64_t(m_dram->m_timing_vals("rate")) * 1000000) * 100e-6));

      register_stat(rb_miss).name("total_rowbuffer_misses");
      register_stat(rb_hits).name("total_rowbuffer_hits");
      register_stat(last_bank_util).name("total_bytes_processed");
    };

    void update(bool request_found, ReqBuffer::iterator& req_it) override {
      if (request_found) {

        if(m_dram->m_command_meta(req_it->command).is_accessing)
        {
          rb_hits++;
        }
        std::string_view command = m_dram->m_commands(req_it->command);
        if(m_dram->m_command_meta(req_it->command).is_opening && m_dram->m_command_scopes(req_it->command) == m_dram->m_levels("row")) //opened row
        {
          rb_miss++;
          uint64_t bank_count = m_dram->get_level_size("bank");
          int type = req_it->type_id == Ramulator::Request::Type::Write ? RH_WRITE : RH_READ;
          //std::cout << m_dram->m_levels("row") << "\n";
          HC.log_charge(Address(req_it->addr_vec[m_dram->m_levels("channel")], req_it->addr_vec[m_dram->m_levels("bank")] + bank_count*req_it->addr_vec[m_dram->m_levels("bankgroup")], req_it->addr_vec[m_dram->m_levels("rank")],req_it->addr_vec[m_dram->m_levels("row")]),req_it->addr,0,type,req_it->source_id == 1,HC.total_cycles,req_it->type_id == Ramulator::Request::Type::Write);
        }
        else if(m_dram->m_command_meta(req_it->command).is_refreshing && m_dram->m_command_scopes(req_it->command) == m_dram->m_levels("rank")) //refreshed
        {
          uint64_t bank_count = m_dram->get_level_size("bank");
          HC.log_refresh(Address(req_it->addr_vec[m_dram->m_levels("channel")], req_it->addr_vec[m_dram->m_levels("bank")] + bank_count*req_it->addr_vec[m_dram->m_levels("bankgroup")], req_it->addr_vec[m_dram->m_levels("rank")],req_it->addr_vec[m_dram->m_levels("row")]));
        }
      }
      HC.log_cycle();
      HC.is_start_cycle(HC.total_cycles);

    if (HC.total_cycles % HC.cycles_per_heartbeat == 0) {
    // print heartbeat
    uint64_t bank_util = HammerCounter::processed_packets * 64;
    double cum_hit_rate = ((rb_hits - rb_miss) / double(rb_hits));
    double hit_rate = ((rb_hits - last_rb_hits - rb_miss + last_rb_miss) / double((rb_hits-last_rb_hits)));

    double throughput = (((bank_util - last_bank_util)/double(HC.cycles_per_heartbeat)) * (double)(uint64_t(m_dram->m_timing_vals("rate"))*1e6)) / double(1<<30);
    double cum_throughput = (((bank_util)/double(HC.total_cycles)) * (double)(uint64_t(m_dram->m_timing_vals("rate"))*1e6)) / double(1<<30);
    printf("Heartbeat DRAM Throughput: %.3fGiB/s Cumulative Throughput: %.3fGiB/s Row Buffer Hit Rate: %.3f Cumulative Row Buffer Hit Rate: %.3f\n",throughput,cum_throughput,hit_rate,cum_hit_rate);
    last_bank_util = bank_util;
    last_rb_hits = rb_hits;
    last_rb_miss = rb_miss;
    }
    };

    void finalize() override {
      HC.print_file();
    }

};
}
#endif

#endif
