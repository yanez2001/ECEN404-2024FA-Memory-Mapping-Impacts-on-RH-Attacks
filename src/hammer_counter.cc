#include "../inc/hammer_counter.h"
#include <iostream>


std::string HammerCounter::output_f = "hammer";
uint64_t HammerCounter::processed_packets = 0;
uint64_t HammerCounter::target_row = 0;
uint64_t HammerCounter::target_cycle = 0;

std::ofstream memory_log_dump;

std::string HammerCounter::get_output_file() { return (output_f); }

uint64_t HammerCounter::get_target_row() { return (target_row); }

void HammerCounter::set_target_row(uint64_t row) { target_row = row; }

void HammerCounter::set_target_cycle(uint64_t cycle) { target_cycle = cycle; }

HighestCount::HighestCount()
{
  hammer_count = 0;
  prefetch_hammer_count = 0;
  normal_hammer_count = 0;
  truncated_by_refresh = false;
  hammered_besides_refresh = false;
  start_cycle = 0;
  write_back_count = 0;
}
LifetimeCount::LifetimeCount()
{
  total_hammers = 0;
  normal_hammers = 0;
  prefetch_hammers = 0;
  is_refresh_only = true;
  lost_hammers_to_access = 0;
  lost_hammers_to_refresh = 0;
  write_back_hammers = 0;
}
void Count::CopyInto(Count C)
{
  //replace if new highest hammer count
  if(C.highest.hammer_count > highest.hammer_count)
  {
    highest = C.highest;
  }
  //global stats
  Combine(C);
}
void Count::Combine(Count C)
{
  lifetime.total_hammers += C.highest.hammer_count;
  lifetime.normal_hammers += C.highest.normal_hammer_count;
  lifetime.prefetch_hammers += C.highest.prefetch_hammer_count;
  lifetime.write_back_hammers += C.highest.write_back_count;
  lifetime.is_refresh_only = lifetime.is_refresh_only && !C.highest.hammered_besides_refresh;
  if(C.highest.truncated_by_refresh)
    lifetime.lost_hammers_to_refresh += C.highest.hammer_count;
  else
    lifetime.lost_hammers_to_access += C.highest.hammer_count;
}
void HammerCounter::InsertIntoMap(Address A, Count C)
{
  if (row_open_master.find(A) != row_open_master.end()) {
    //entry already exists, copy into
    row_open_master[A].CopyInto(C);
  } else {
    //need to create a new entry in the master table
    row_open_master[A] = Count();
    row_open_master[A].CopyInto(C);
  }
}

Count::Count()
{
  lifetime = LifetimeCount();
  highest = HighestCount();
}
bool Count::operator<(Count& b) { return (highest.hammer_count < b.highest.hammer_count); }

HammerCounter::HammerCounter()
{
  dram_rows = DRAM_ROWS;
  dram_columns = DRAM_COLUMNS;
  dram_ranks = DRAM_RANKS;
  dram_banks = DRAM_BANKS;
  dram_channels = DRAM_CHANNELS;

  refresh_row = 0;
  row_charges_r = 0;
  row_charges_rn = 0;
  row_charges_rp = 0;
  row_charges_w = 0;
  row_charges_wn = 0;
  row_charges_wp = 0;
  refreshes = 0;
  refresh_cycles = 0;
  total_cycles = 0;
  row_charges_ref = 0;

  lost_hammers = 0;
  lost_hammers_to_access = 0;
  lost_hammers_to_refresh = 0;
  victim_reads = 0;
  victim_writes = 0;

  highest_hammers_per_cycle = 0;
  highest_hammers_per_cycle_p = 0;
  highest_hammer_row = 0;
  last_hammer_cycles = 0;

  channel_num = 0;
  phase = 0;

  cycles_per_bin = uint64_t((DRAM_IO_FREQ * 1000000) * 100e-6); // 100 us bins

  cycles_per_heartbeat = (STAT_PRINTING_PERIOD / 4);
}

uint64_t previous_row = 0;
uint64_t previous_addr = 0;
uint64_t previous_vaddr = 0;
uint64_t previous_cycle = 0;
uint64_t previous_bank = 0;
int previous_type = 0;
bool previous_pref = false;
bool previous_wb = false;

void HammerCounter::log_refresh(Address addr)
{
  //do refresh based off of first rank
  if(addr.get_rank() == 0)
  {
    for(int k = 0; k < (dram_ranks); k++)
    {
      for(int i = 0; i < (dram_rows)/(1<<13); i++)
      {
        for(int j = 0; j < dram_banks; j++)
        {
          Address addr2(addr.get_channel(),j,k,refresh_row+i);
          log_charge(addr2,0,0,RH_REFRESH,false,0,false);
        }
      }
    }
    refreshes+=dram_banks;
    refresh_row += (dram_rows)/(1<<13);
    if(refresh_row >= dram_rows)
    {
      refresh_cycles++;
      refresh_row = 0;
    }
  }
    
}
void HammerCounter::log_charge(Address addr,uint64_t p_addr, uint64_t v_addr, int type, bool prefetch, uint64_t cycle, bool write_back)
{
  channel_num = addr.get_channel();
  // log hit on both adjacent rows
  //std::cout << addr.get_row() << "\n";
  Address addr_high = Address(addr.get_channel(), addr.get_bank(), addr.get_rank(), (addr.get_row() + 1) % dram_rows);
  Address addr_low = Address(addr.get_channel(), addr.get_bank(), addr.get_rank(), (addr.get_row() == 0) ? dram_rows - 1 : addr.get_row() - 1);

  if(addr.get_row() == target_row || addr_high.get_row() == target_row || addr_low.get_row() == target_row)
  {
    //print to mem file
    memory_log_dump.open(output_f + "_memory_accesses.log",std::fstream::app);
          if(previous_pref)
            memory_log_dump << "PPREFETCH ";
          else if(previous_wb)
            memory_log_dump << "PWRITEBACK ";
          else
            memory_log_dump << "PNORMAL ";
          memory_log_dump << "PADDR: " << std::hex << previous_addr << 
                             " VADDR: " << std::hex << previous_vaddr << 
                             " PROW: " << previous_row << 
                             " PBANK: " << previous_bank <<
                             " CYCLE: " << std::dec << previous_cycle << std::endl;
          if(prefetch)
            memory_log_dump << "PREFETCH ";
          else if(write_back)
            memory_log_dump << "WRITEBACK ";
          else
            memory_log_dump << "NORMAL ";
          memory_log_dump << "PADDR: " << std::hex << p_addr << 
                             " VADDR: " << v_addr << 
                             " PROW: " << addr.get_row() <<
                             " PBANK: " << addr.get_bank() << 
                             " CYCLE: " << std::dec << total_cycles << std::endl;
    memory_log_dump.close();

  }
  previous_row = addr.get_row();
  previous_addr = p_addr;
  previous_vaddr = v_addr;
  previous_bank = addr.get_bank();
  previous_cycle = total_cycles;
  previous_type = type;
  previous_pref = prefetch;
  previous_wb = write_back;

  #ifdef RAMULATOR
  #else
 if(type == RH_REFRESH)
  {
    refreshes++;
    refresh_row = addr.get_row();
    if(refresh_row + 1 == DRAM_ROWS)
    refresh_cycles++;
  }
  #endif

  // low row
  if (addr.get_row() != 0) {
    if (row_open_counter.find(addr_low) == row_open_counter.end())
    {
      row_open_counter[addr_low] = Count();
      row_open_counter[addr_low].highest.start_cycle = cycle;
    }
    row_open_counter[addr_low].highest.hammer_count++;

    if(type != RH_REFRESH)
    row_open_counter[addr_low].highest.hammered_besides_refresh = true;

    //writeback, prefetch, or normal
    if (prefetch)
      row_open_counter[addr_low].highest.prefetch_hammer_count++;
    else if(write_back)
      row_open_counter[addr_low].highest.write_back_count++;
    else
      row_open_counter[addr_low].highest.normal_hammer_count++;

    if (row_open_counter[addr_low].highest.hammer_count > highest_hammers_per_cycle) {
      highest_hammers_per_cycle = row_open_counter[addr_low].highest.hammer_count;
      highest_hammers_per_cycle_p = row_open_counter[addr_low].highest.prefetch_hammer_count;
      highest_hammers_per_cycle_wb = row_open_counter[addr_low].highest.write_back_count;
      highest_hammer_row = addr_low.get_row();
    }

    if (type == RH_READ) {
      row_charges_r++;
      if (prefetch)
        row_charges_rp++;
      else
        row_charges_rn++;
    } else if(type == RH_WRITE) {
      row_charges_w++;
      if (prefetch)
        row_charges_wp++;
      else if(write_back)
        row_charges_wb++;
      else
        row_charges_wn++;
    }
    else
    {
      row_charges_ref++;
    }

    // for output histogram
    if (type == RH_READ && !prefetch) {
      if (read_access_histogram.find(total_cycles / cycles_per_bin) == read_access_histogram.end())
        read_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        read_access_histogram[total_cycles / cycles_per_bin]++;
    }
    if (type == RH_READ && prefetch)
    {
      if (pref_access_histogram.find(total_cycles / cycles_per_bin) == pref_access_histogram.end())
        pref_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        pref_access_histogram[total_cycles / cycles_per_bin]++;
    } 
    if (type == RH_WRITE && write_back){
      if (wb_access_histogram.find(total_cycles / cycles_per_bin) == wb_access_histogram.end())
        wb_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        wb_access_histogram[total_cycles / cycles_per_bin]++;
    }
    if (type == RH_WRITE && !write_back)
    {
      if (write_access_histogram.find(total_cycles / cycles_per_bin) == write_access_histogram.end())
        write_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        write_access_histogram[total_cycles / cycles_per_bin]++;
    }
  }

  // high row
  if (addr.get_row() != (dram_rows - 1)) {
    if (row_open_counter.find(addr_high) == row_open_counter.end())
    {
      row_open_counter[addr_high] = Count();
      row_open_counter[addr_high].highest.start_cycle = cycle;
    }
    row_open_counter[addr_high].highest.hammer_count++;

    if(type != RH_REFRESH)
    row_open_counter[addr_high].highest.hammered_besides_refresh = true;

    if (prefetch)
      row_open_counter[addr_high].highest.prefetch_hammer_count++;
    else if (write_back)
      row_open_counter[addr_high].highest.write_back_count++;
    else
      row_open_counter[addr_high].highest.normal_hammer_count++;

    if (row_open_counter[addr_high].highest.hammer_count > highest_hammers_per_cycle) {
      highest_hammers_per_cycle = row_open_counter[addr_high].highest.hammer_count;
      highest_hammers_per_cycle_p = row_open_counter[addr_high].highest.prefetch_hammer_count;
      highest_hammers_per_cycle_wb = row_open_counter[addr_high].highest.write_back_count;
      highest_hammer_row = addr_high.get_row();
    }

    if (type == RH_READ) {
      row_charges_r++;
      if (prefetch)
        row_charges_rp++;
      else
        row_charges_rn++;
    } else if(type == RH_WRITE){
      row_charges_w++;
      if (prefetch)
        row_charges_wp++;
      else if(write_back)
        row_charges_wb++;
      else
        row_charges_wn++;
    }
    else
    {
      row_charges_ref++;
    }

    // for output histogram
    if (type == RH_READ && !prefetch) {
      if (read_access_histogram.find(total_cycles / cycles_per_bin) == read_access_histogram.end())
        read_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        read_access_histogram[total_cycles / cycles_per_bin]++;
    }
    if (type == RH_READ && prefetch)
    {
      if (pref_access_histogram.find(total_cycles / cycles_per_bin) == pref_access_histogram.end())
        pref_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        pref_access_histogram[total_cycles / cycles_per_bin]++;
    } 
    if (type == RH_WRITE && write_back){
      if (wb_access_histogram.find(total_cycles / cycles_per_bin) == wb_access_histogram.end())
        wb_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        wb_access_histogram[total_cycles / cycles_per_bin]++;
    }
    if (type == RH_WRITE && !write_back)
    {
      if (write_access_histogram.find(total_cycles / cycles_per_bin) == write_access_histogram.end())
        write_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        write_access_histogram[total_cycles / cycles_per_bin]++;
    }
  }

  // the secondary impact of this access is that the current hammers on the row_addr are truncated here
  if (row_open_counter.find(addr) != row_open_counter.end()) {
    row_open_counter[addr].highest.truncated_by_refresh = (type == RH_REFRESH);
    InsertIntoMap(addr, row_open_counter[addr]);

    // increment lost hammers
    lost_hammers += row_open_counter[addr].highest.hammer_count;
    if(type == RH_REFRESH)
      lost_hammers_to_refresh += row_open_counter[addr].highest.hammer_count;
    else
      lost_hammers_to_access += row_open_counter[addr].highest.hammer_count;
    row_open_counter.erase(addr);

    // did we read from a victim or write to a victim?
    if (type == RH_READ)
      victim_reads++;
    else if(type == RH_WRITE)
      victim_writes++;
  }
}

/*void HammerCounter::log_refresh(uint64_t row, uint64_t cycle)
{
  refresh_row = row;
  refreshes++;
  std::cout << "test\n";
  for (uint64_t i = 0; i < rows_per_refresh; i++) {
      std::map<Address, Count> row_open_counter_temp;
      // check to see if row was accessed at all
      auto it = row_open_counter.begin();
      while (it != row_open_counter.end()) {
        if (it->first.get_row() == refresh_row) {
          // retrieve # of row opens
          Count hammers = (*it).second;
          // if exceeds master, overwrite
          hammers.start_cycle = cycle;
          hammers.truncated_by_refresh = true;
          InsertIntoMap(it->first, hammers);
          lost_hammers += hammers.total_hammers;
          lost_hammers_to_refresh += hammers.total_hammers;
          // remove from counter
          it++;
        } else {
          row_open_counter_temp[it->first] = it->second;
          it++;
        }
      }
      row_open_counter = row_open_counter_temp;
      // increment refresh row
      refresh_row++;

      if(refresh_row % DRAM_ROWS == 0)
      {
        refresh_cycles++;
      }
    }
}*/

bool HammerCounter::is_start_cycle(uint64_t cycle)
{
  bool start = (cycle == target_cycle);
  if(start)
  {
    row_open_master.clear();
    row_open_counter.clear();
  }
  return(start);
}

void HammerCounter::log_cycle()
{
  total_cycles++;

  if (total_cycles % cycles_per_heartbeat == 0) {
    // print heartbeat
    std::cout << "Heartbeat HAMMER COUNTER " << channel_num << " : " << (unsigned long)(total_cycles) << " Highest Hammer Row: " << std::hex
              << highest_hammer_row << std::dec << " Hammer Count: " << highest_hammers_per_cycle << " (" << highest_hammers_per_cycle_p
              << ") Refresh Row: " << std::hex << refresh_row << std::dec << " Heartbeat Hammers: " << ((row_charges_r + row_charges_w) - last_hammer_cycles)
              << "\n";
    highest_hammers_per_cycle = 0;
    highest_hammers_per_cycle_p = 0;
    last_hammer_cycles = row_charges_r + row_charges_w;
  }

  //clear for synchronicity
  if(total_cycles == target_cycle)
  {
    row_open_counter.clear();
    row_open_master.clear();
  }
}

void HammerCounter::flush()
{
  for (auto it = row_open_counter.begin(); it != row_open_counter.end(); it++) {
    InsertIntoMap(it->first, it->second);
  }
  row_open_counter.clear();
}

bool sortbysec(const std::pair<Address, Count>& a, const std::pair<Address, Count>& b)
{
  return (a.second.highest.hammer_count > b.second.highest.hammer_count);
}

void HammerCounter::set_output_file(std::string f) { output_f = f; }

void HammerCounter::print_file()
{
  std::string file_name;
  file_name = output_f + "_" + std::to_string(channel_num) + "_" + std::to_string(phase);
  flush();
  // calculate what percentage of each address space was used
  uint64_t unique_rows_visited = 0;
  for(auto it = row_open_master.begin(); it != row_open_master.end(); it++)
  {
    if(!it->second.lifetime.is_refresh_only)
    unique_rows_visited++;
  }
  long double address_space_usage = (unique_rows_visited) / (double)(dram_ranks * dram_banks * dram_rows * dram_channels);
  std::ofstream file;
  file.open(file_name + ".log");
  file << "ROW-HAMMER STATISTICS\n";
  file << "####################################################################################################\n";
  file << "Row Hammers (READ INSTIGATED): " << row_charges_r << "\n";
  file << "\tNormal: " << row_charges_rn << " \tPrefetch: " << row_charges_rp << "\n";
  file << "Row Hammers (WRITE INSTIGATED): " << row_charges_w << "\n";
  file << "\tNormal: " << row_charges_wn << " \tPrefetch: " << row_charges_wp << "\tWriteback: " << row_charges_wb << "\n";
  file << "Row Hammers (REFRESH INSTIGATED): " << row_charges_ref << "\n";
  file << "Total Row Hammers: " << row_charges_r + row_charges_w + row_charges_ref<< "\n";
  file << "####################################################################################################\n";
  file << "Channels: " << dram_channels << "\n";
  file << "Ranks: " << dram_ranks << "\n";
  file << "Banks: " << dram_banks << "\n";
  file << "Rows: " << dram_rows << "\n";
  file << "Columns: " << dram_columns << "\n";
  file << "Address Space Used: " << address_space_usage * 100.0 << "%\n";
  file << "####################################################################################################\n";
  file << "Rows Refreshed: " << refreshes << '\n';
  file << "Refresh Cycles: " << refresh_cycles/dram_banks << '\n';
  file << "####################################################################################################\n";
  file << "Victim Reads: " << victim_reads << "\n";
  file << "Victim Writes: " << victim_writes << "\n";
  file << "Lost Hammers: " << lost_hammers << "\n";
  file << "\tTo Refresh: " << lost_hammers_to_refresh << " \tTo Access: " << lost_hammers_to_access << "\n";
  file << "####################################################################################################\n";
  file << "Stats by Row\n";

  // lets try to sort
  std::vector<std::pair<Address, Count>> final_output;
  for (auto it = row_open_master.begin(); it != row_open_master.end(); it++) {
    final_output.push_back(*(it));
  }
  std::sort(final_output.begin(), final_output.end(), sortbysec);

  // print output
  for (auto it = final_output.begin(); it != final_output.end(); it++) {
    //had to undergo non-refresh activation
    if(!it->second.lifetime.is_refresh_only)
      {
      // Print out data for row
      file << "\tChannel: 0x" << std::hex << it->first.get_channel();
      file << "\tRank: 0x" << std::hex << it->first.get_rank();
      file << "\tBank: 0x" << std::hex << it->first.get_bank();
      file << "\tRow: 0x" << std::hex << it->first.get_row();
      file << "\tLifetime Hammers/(Normal:Prefetch:Writeback): " << std::dec << it->second.lifetime.total_hammers << " (" << it->second.lifetime.normal_hammers << ":" << it->second.lifetime.prefetch_hammers << ":" << it->second.lifetime.write_back_hammers << ") ";
      if (it->second.highest.truncated_by_refresh)
        file << "\tLimit: Refresh ";
      else
        file << "\tLimit: Access ";
      file << "\tHighest Single-Cycle Hammers(" << std::dec << it->second.highest.start_cycle << ")/(Normal:Prefetch:Writeback): " << std::dec << it->second.highest.hammer_count << " (" << it->second.highest.normal_hammer_count << ":" << it->second.highest.prefetch_hammer_count
          << ":" << it->second.highest.write_back_count << ") ";
      file << "\tLost Hammers/(Refresh:Access): " << std::dec << it->second.lifetime.lost_hammers_to_refresh + it->second.lifetime.lost_hammers_to_access << " ("
          << it->second.lifetime.lost_hammers_to_refresh << ":" << it->second.lifetime.lost_hammers_to_access << ") ";
      file << "\n";
    }
  }

  file << "####################################################################################################\n";
  file.close();

  // print histograms now
  std::ofstream file_hr;
  file_hr.open(file_name + ".hr");
  for (auto it = read_access_histogram.begin(); it != read_access_histogram.end(); it++) {
    file_hr << (it->first * 100) << " " << it->second << "\n";
  }
  file_hr.close();
  std::ofstream file_hw;
  file_hw.open(file_name + ".hw");
  for (auto it = write_access_histogram.begin(); it != write_access_histogram.end(); it++) {
    file_hw << (it->first * 100) << " " << it->second << "\n";
  }
  file_hw.close();
  std::ofstream file_hp;
  file_hp.open(file_name + ".hp");
  for (auto it = pref_access_histogram.begin(); it != pref_access_histogram.end(); it++) {
    file_hp << (it->first * 100) << " " << it->second << "\n";
  }
  file_hp.close();
  std::ofstream file_hwb;
  file_hwb.open(file_name + ".hwb");
  for (auto it = wb_access_histogram.begin(); it != wb_access_histogram.end(); it++) {
    file_hwb << (it->first * 100) << " " << it->second << "\n";
  }
  file_hwb.close();

  phase++;
}
