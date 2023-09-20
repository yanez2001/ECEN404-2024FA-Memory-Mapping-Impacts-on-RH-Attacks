#include "../inc/hammer_counter.h"
#include <iostream>

std::string HammerCounter::output_f = "hammer";

uint64_t HammerCounter::target_row = 0;
uint64_t HammerCounter::target_cycle = 0;

std::string HammerCounter::get_output_file() { return (output_f); }

uint64_t HammerCounter::get_target_row() { return (target_row); }

void HammerCounter::set_target_row(uint64_t row) { target_row = row; }

void HammerCounter::set_target_cycle(uint64_t cycle) { target_cycle = cycle; }

void HammerCounter::InsertIntoMap(Address A, Count C)
{
  if (row_open_master.find(A) != row_open_master.end()) {
    if (C.total_hammers > row_open_master[A].highest_hammer_count) {
      row_open_master[A].highest_hammer_count = C.total_hammers;
      row_open_master[A].highest_prefetch_hammer_count = C.prefetch_hammers;
      row_open_master[A].highest_normal_hammer_count = C.normal_hammers;
      row_open_master[A].truncated_by_refresh = C.truncated_by_refresh;
      row_open_master[A].start_cycle = C.start_cycle;
    }
    row_open_master[A].total_hammers += C.total_hammers;
    row_open_master[A].normal_hammers += C.normal_hammers;
    row_open_master[A].prefetch_hammers += C.prefetch_hammers;
    row_open_master[A].is_refresh_only = row_open_master[A].is_refresh_only && C.is_refresh_only;
    if (C.truncated_by_refresh)
      row_open_master[A].lost_hammers_to_refresh += C.total_hammers;
    else
      row_open_master[A].lost_hammers_to_access += C.total_hammers;
  } else {
    if(!C.is_refresh_only)
    unique_rows_visited++;
    row_open_master[A] = Count();
    row_open_master[A].highest_hammer_count = C.total_hammers;
    row_open_master[A].highest_prefetch_hammer_count = C.prefetch_hammers;
    row_open_master[A].highest_normal_hammer_count = C.normal_hammers;
    row_open_master[A].truncated_by_refresh = C.truncated_by_refresh;
    row_open_master[A].total_hammers = C.total_hammers;
    row_open_master[A].normal_hammers = C.normal_hammers;
    row_open_master[A].prefetch_hammers = C.prefetch_hammers;
    row_open_master[A].start_cycle = C.start_cycle;
    row_open_master[A].is_refresh_only = C.is_refresh_only;
    if (C.truncated_by_refresh)
      row_open_master[A].lost_hammers_to_refresh = C.total_hammers;
    else
      row_open_master[A].lost_hammers_to_access = C.total_hammers;
  }
}

Count::Count()
{
  lost_hammers_to_refresh = 0;
  lost_hammers_to_access = 0;

  total_hammers = 0;
  normal_hammers = 0;
  prefetch_hammers = 0;

  highest_hammer_count = 0;
  highest_prefetch_hammer_count = 0;
  highest_normal_hammer_count = 0;

  start_cycle = 0;

  truncated_by_refresh = true;
  is_refresh_only = true;
}
bool Count::operator<(Count& b) { return (highest_hammer_count < b.highest_hammer_count); }

HammerCounter::HammerCounter()
{
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
  unique_rows_visited = 0;

  channel_num = 0;
  phase = 0;
  // the entire memory space needs refreshed in 64ms.
  // we run at DRAM_IO_FREQ and have to cover DRAM_ROWS in that time
  // how often do we need to sim a refresh?
  rows_per_refresh = 8;
  
  cycles_per_refresh = ((7.8e-6) * DRAM_IO_FREQ * 1000000);

  cycles_per_bin = (cycles_per_refresh / 7.8) * 100; // 100 us bins

  cycles_per_heartbeat = (STAT_PRINTING_PERIOD / 4) * ((double)DRAM_IO_FREQ / (double)CPU_FREQ);
}

void HammerCounter::log_charge(Address addr, int type, bool prefetch, uint64_t cycle)
{
  channel_num = addr.get_channel();
  // log hit on both adjacent rows
  Address addr_high = Address(addr.get_channel(), addr.get_bank(), addr.get_rank(), (addr.get_row() + 1) % DRAM_ROWS);
  Address addr_low = Address(addr.get_channel(), addr.get_bank(), addr.get_rank(), (addr.get_row() == 0) ? DRAM_ROWS - 1 : addr.get_row() - 1);

  if(type == RH_REFRESH)
  {
    refreshes++;
    refresh_row = addr.get_row();
    if(refresh_row + 1 == DRAM_ROWS)
    refresh_cycles++;
  }

  // low row
  if (addr.get_row() != 0) {
    if (row_open_counter.find(addr_low) == row_open_counter.end())
      row_open_counter[addr_low] = Count();
    row_open_counter[addr_low].total_hammers++;
    row_open_counter[addr_low].start_cycle = cycle;
    if(type != RH_REFRESH)
    row_open_counter[addr_low].is_refresh_only = false;
    if (prefetch)
      row_open_counter[addr_low].prefetch_hammers++;
    else
      row_open_counter[addr_low].normal_hammers++;
    if (row_open_counter[addr_low].total_hammers > highest_hammers_per_cycle) {
      highest_hammers_per_cycle = row_open_counter[addr_low].total_hammers;
      highest_hammers_per_cycle_p = row_open_counter[addr_low].prefetch_hammers;
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
      else
        row_charges_wn++;
    }
    else
    {
      row_charges_ref++;
    }

    // for output histogram
    if (type == RH_READ) {
      if (read_access_histogram.find(total_cycles / cycles_per_bin) == read_access_histogram.end())
        read_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        read_access_histogram[total_cycles / cycles_per_bin]++;
    } else {
      if (write_access_histogram.find(total_cycles / cycles_per_bin) == write_access_histogram.end())
        write_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        write_access_histogram[total_cycles / cycles_per_bin]++;
    }
  }

  // high row
  if (addr.get_row() != (DRAM_ROWS - 1)) {
    if (row_open_counter.find(addr_high) == row_open_counter.end())
      row_open_counter[addr_high] = Count();
    row_open_counter[addr_high].total_hammers++;
    row_open_counter[addr_high].start_cycle = cycle;
    if(type != RH_REFRESH)
    row_open_counter[addr_high].is_refresh_only = false;
    if (prefetch)
      row_open_counter[addr_high].prefetch_hammers++;
    else
      row_open_counter[addr_high].normal_hammers++;

    if (row_open_counter[addr_high].total_hammers > highest_hammers_per_cycle) {
      highest_hammers_per_cycle = row_open_counter[addr_high].total_hammers;
      highest_hammers_per_cycle_p = row_open_counter[addr_high].prefetch_hammers;
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
      else
        row_charges_wn++;
    }
    else
    {
      row_charges_ref++;
    }

    // for output histogram
    if (type == RH_READ) {
      if (read_access_histogram.find(total_cycles / cycles_per_bin) == read_access_histogram.end())
        read_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        read_access_histogram[total_cycles / cycles_per_bin]++;
    } else if(type == RH_WRITE) {
      if (write_access_histogram.find(total_cycles / cycles_per_bin) == write_access_histogram.end())
        write_access_histogram[total_cycles / cycles_per_bin] = 1;
      else
        write_access_histogram[total_cycles / cycles_per_bin]++;
    }
  }

  // the secondary impact of this access is that the current hammers on the row_addr are truncated here
  if (row_open_counter.find(addr) != row_open_counter.end()) {
    row_open_counter[addr].truncated_by_refresh = (type == RH_REFRESH);
    row_open_counter[addr].start_cycle = cycle;
    InsertIntoMap(addr, row_open_counter[addr]);

    // increment lost hammers
    lost_hammers += row_open_counter[addr].total_hammers;
    if(type == RH_REFRESH)
      lost_hammers_to_refresh += row_open_counter[addr].total_hammers;
    else
      lost_hammers_to_access += row_open_counter[addr].total_hammers;
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
    std::cout << "Heartbeat DRAM cycle: " << (unsigned long)(total_cycles * ((double)CPU_FREQ / (double)DRAM_IO_FREQ)) << " Highest Hammer Row: " << std::hex
              << highest_hammer_row << std::dec << " Hammer Count: " << highest_hammers_per_cycle << " (" << highest_hammers_per_cycle_p
              << ") Refresh Row: " << std::hex << refresh_row << std::dec << " Heartbeat Hammers: " << ((row_charges_r + row_charges_w) - last_hammer_cycles)
              << "\n";
    highest_hammers_per_cycle = 0;
    highest_hammers_per_cycle_p = 0;
    last_hammer_cycles = row_charges_r + row_charges_w;
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
  return (a.second.highest_hammer_count > b.second.highest_hammer_count);
}

void HammerCounter::set_output_file(std::string f) { output_f = f; }

void HammerCounter::print_file()
{
  std::string file_name;
  file_name = output_f + "_" + std::to_string(channel_num) + "_" + std::to_string(phase);
  flush();
  // calculate what percentage of each address space was used
  unique_rows_visited = 0;
  for(auto it = row_open_master.begin(); it != row_open_master.end(); it++)
  {
    if(!it->second.is_refresh_only)
    unique_rows_visited++;
  }
  long double address_space_usage = (unique_rows_visited) / (double)(DRAM_RANKS * DRAM_BANKS * DRAM_ROWS);
  std::ofstream file;
  file.open(file_name + ".log");
  file << "ROW-HAMMER STATISTICS\n";
  file << "####################################################################################################\n";
  file << "Row Hammers (READ INSTIGATED): " << row_charges_r << "\n";
  file << "\tNormal: " << row_charges_rn << " \tPrefetch: " << row_charges_rp << "\n";
  file << "Row Hammers (WRITE INSTIGATED): " << row_charges_w << "\n";
  file << "\tNormal: " << row_charges_wn << " \tPrefetch: " << row_charges_wp << "\n";
  file << "Row Hammers (REFRESH INSTIGATED): " << row_charges_ref << "\n";
  file << "Total Row Hammers: " << row_charges_r + row_charges_w + row_charges_ref<< "\n";
  file << "####################################################################################################\n";
  file << "Channels: " << DRAM_CHANNELS << "\n";
  file << "Ranks: " << DRAM_RANKS << "\n";
  file << "Banks: " << DRAM_BANKS << "\n";
  file << "Rows: " << DRAM_ROWS << "\n";
  file << "Columns: " << DRAM_COLUMNS << "\n";
  file << "Address Space Used: " << address_space_usage * 100.0 << "%\n";
  file << "####################################################################################################\n";
  file << "Rows Refreshed: " << refreshes << '\n';
  file << "Refresh Cycles: " << refresh_cycles << '\n';
  file << "Rows per Refresh: " << rows_per_refresh << '\n';
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
    // Print out data for row
    file << "\tChannel: 0x" << std::hex << it->first.get_channel();
    file << "\tRank: 0x" << std::hex << it->first.get_rank();
    file << "\tBank: 0x" << std::hex << it->first.get_bank();
    file << "\tRow: 0x" << std::hex << it->first.get_row();
    file << "\tLifetime Hammers/(Prefetch): " << std::dec << it->second.total_hammers << " (" << it->second.prefetch_hammers << ") ";
    if (it->second.truncated_by_refresh)
      file << "\tLimit: Refresh ";
    else
      file << "\tLimit: Access ";
    file << "\tHighest Single-Cycle Hammers(" << std::dec << it->second.start_cycle << ")/(Prefetch): " << std::dec << it->second.highest_hammer_count << " (" << it->second.highest_prefetch_hammer_count
         << ") ";
    file << "\tLost Hammers/(Access): " << std::dec << it->second.lost_hammers_to_refresh + it->second.lost_hammers_to_access << " ("
         << it->second.lost_hammers_to_access << ") ";
    file << "\n";
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

  phase++;
}
