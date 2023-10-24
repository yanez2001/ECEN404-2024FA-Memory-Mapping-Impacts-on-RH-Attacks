/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "vmem.h"

#include <cassert>

#include "champsim.h"
#include "champsim_constants.h"
#include "dram_controller.h"
#include <fmt/core.h>

uint64_t VirtualMemory::virtual_seed = 0;

void VirtualMemory::set_virtual_seed(uint64_t v_seed)
{
  virtual_seed = v_seed;
}

VirtualMemory::VirtualMemory(uint64_t page_table_page_size, std::size_t page_table_levels, uint64_t minor_penalty, MEMORY_CONTROLLER& _dram)
    : next_ppage(VMEM_RESERVE_CAPACITY), last_ppage(1ull << (LOG2_PAGE_SIZE + champsim::lg2(page_table_page_size / PTE_BYTES) * page_table_levels)),
      minor_fault_penalty(minor_penalty), pt_levels(page_table_levels), pte_page_size(page_table_page_size),pmem_size(_dram.size()), dram(_dram)
{
  assert(page_table_page_size > 1024);
  assert(page_table_page_size == (1ull << champsim::lg2(page_table_page_size)));
  assert(last_ppage > VMEM_RESERVE_CAPACITY);

  auto required_bits = champsim::lg2(last_ppage);
  if (required_bits > 64)
    fmt::print("WARNING: virtual memory configuration would require {} bits of addressing.\n", required_bits); // LCOV_EXCL_LINE
  if (required_bits > champsim::lg2(dram.size()))
    fmt::print("WARNING: physical memory size is smaller than virtual memory size; Virtual address space will be aliased.\n"); // LCOV_EXCL_LINE

  populate_pages();
}
void VirtualMemory::shuffle_pages()
{
  if(virtual_seed != 0)
  {
    std::mt19937_64 rng(virtual_seed);
    std::shuffle(ppage_free_list.begin(),ppage_free_list.end(),rng);
    fmt::print("Shuffled {} physical pages with seed {}\n",ppage_free_list.size(),virtual_seed);
  }
}
void VirtualMemory::populate_pages()
{
  ppage_free_list.resize((pmem_size - VMEM_RESERVE_CAPACITY)/PAGE_SIZE);
  uint64_t base_address = VMEM_RESERVE_CAPACITY;
  for(auto it = ppage_free_list.begin(); it != ppage_free_list.end(); it++)
  {
    *(it) = base_address;
    base_address += PAGE_SIZE;
  }
  fmt::print("Created {} new physical pages\n",ppage_free_list.size());
}
uint64_t VirtualMemory::shamt(std::size_t level) const { return LOG2_PAGE_SIZE + champsim::lg2(pte_page_size / PTE_BYTES) * (level - 1); }

uint64_t VirtualMemory::get_offset(uint64_t vaddr, std::size_t level) const
{
  return (vaddr >> shamt(level)) & champsim::bitmask(champsim::lg2(pte_page_size / PTE_BYTES));
}

uint64_t VirtualMemory::ppage_front() const
{
  assert(available_ppages() > 0);
  return ppage_free_list.front();
}

void VirtualMemory::ppage_pop()
{ 
  ppage_free_list.pop_front();
  if(available_ppages() == 0)
  {
    populate_pages();
    shuffle_pages();
  }
}

std::size_t VirtualMemory::available_ppages() const { return ppage_free_list.size(); }

std::pair<uint64_t, uint64_t> VirtualMemory::va_to_pa(uint32_t cpu_num, uint64_t vaddr)
{
  auto [ppage, fault] = vpage_to_ppage_map.insert({{cpu_num, vaddr >> LOG2_PAGE_SIZE}, ppage_front()});

  // this vpage doesn't yet have a ppage mapping
  if (fault)
    ppage_pop();

  auto paddr = champsim::splice_bits(ppage->second, vaddr, LOG2_PAGE_SIZE);

  if constexpr (champsim::debug_print) {
    fmt::print("[VMEM] {} paddr: {:x} vaddr: {:x} fault: {}\n", __func__, paddr, vaddr, fault);
  }

  return {paddr, fault ? minor_fault_penalty : 0};
}


std::pair<uint64_t, uint64_t> VirtualMemory::get_pte_pa(uint32_t cpu_num, uint64_t vaddr, std::size_t level)
{
  if (next_pte_page == 0) {
    next_pte_page = ppage_front();
    ppage_pop();
  }

  std::tuple key{cpu_num, vaddr >> shamt(level), level};
  auto [ppage, fault] = page_table.insert({key, next_pte_page});

  // this PTE doesn't yet have a mapping
  if (fault) {
    next_pte_page += pte_page_size;
    if (!(next_pte_page % PAGE_SIZE)) {
      next_pte_page = ppage_front();
      ppage_pop();
    }
  }

  auto offset = get_offset(vaddr, level);
  auto paddr = champsim::splice_bits(ppage->second, offset * PTE_BYTES, champsim::lg2(pte_page_size));


  if constexpr (champsim::debug_print) {
    fmt::print("[VMEM] {} paddr: {:x} vaddr: {:x} pt_page_offset: {} translation_level: {} fault: {}\n", __func__, paddr, vaddr, offset, level, fault);
  }

  return {paddr, fault ? minor_fault_penalty : 0};
}
