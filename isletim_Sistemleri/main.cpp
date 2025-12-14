/*
 CEMAL EREN ALDIBAÞ
 20232013071
 */

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <map>
#include <cmath>
#include <deque>
#include <limits>
#include <cstdlib> 
#include <thread>
#include <mutex>

using namespace std;

// Konsol yazýlarýný kilitlemek için mutex
mutex print_mutex;

// --- YAPILANDIRMA SABÝTLERÝ ---
const double CONTEXT_SWITCH_TIME = 0.001;
const double RR_QUANTUM = 4.0;
const vector<int> THROUGHPUT_TIMES = {50, 100, 150, 200};

// --- VERÝ YAPILARI ---

struct Process {
    string id;
    double arrival_time;
    double burst_time;
    int priority; // 1=High, 2=Normal, 3=Low
    
    double remaining_time;
    double start_time;
    double completion_time;
    bool is_completed;
    int original_index; 

    Process(string pid, double arr, double burst, string prio_str, int idx) {
        id = pid;
        arrival_time = arr;
        burst_time = burst;
        remaining_time = burst;
        start_time = -1;
        completion_time = 0;
        is_completed = false;
        original_index = idx;

        prio_str.erase(remove_if(prio_str.begin(), prio_str.end(), ::isspace), prio_str.end());
        
        if (prio_str == "high") priority = 1;
        else if (prio_str == "normal") priority = 2;
        else priority = 3;
    }
};

struct GanttSegment {
    double start;
    double end;
    string process_id;
};

struct Metrics {
    double max_wt, avg_wt;
    double max_tat, avg_tat;
    map<int, int> throughput;
    double cpu_utilization;
    int context_switches;
};

// --- YARDIMCI FONKSÝYONLAR ---

void safe_print(const string& msg) {
    lock_guard<mutex> lock(print_mutex);
    cout << msg << endl;
}

vector<Process> load_processes_from_string(const string& csv_data) {
    vector<Process> processes;
    stringstream ss(csv_data);
    string line;
    
    getline(ss, line); 

    int idx = 0;
    while (getline(ss, line)) {
        if (line.empty()) continue;
        stringstream ls(line);
        string segment;
        vector<string> parts;
        
        while (getline(ls, segment, ',')) {
            parts.push_back(segment);
        }

        if (parts.size() >= 4) {
            double arrival = atof(parts[1].c_str());
            double burst = atof(parts[2].c_str());
            processes.emplace_back(parts[0], arrival, burst, parts[3], idx++);
        }
    }
    return processes;
}

vector<Process> load_processes_from_file(const string& filename) {
    ifstream file(filename.c_str());
    if (!file.is_open()) {
        safe_print("HATA: Dosya acilamadi: " + filename);
        return vector<Process>(); 
    }
    
    stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    return load_processes_from_string(buffer.str());
}

Metrics calculate_metrics(const vector<Process>& procs, const vector<GanttSegment>& gantt) {
    Metrics m;
    m.max_wt = 0; m.avg_wt = 0;
    m.max_tat = 0; m.avg_tat = 0;
    m.context_switches = 0;
    
    double total_wt = 0;
    double total_tat = 0;

    vector<double> completion_times;
    size_t completed_procs_count = 0;

    for (size_t i = 0; i < procs.size(); ++i) {
        const Process& p = procs[i];
        if (!p.is_completed) continue;
        
        completed_procs_count++;
        double tat = p.completion_time - p.arrival_time;
        double wt = tat - p.burst_time;
        
        total_tat += tat;
        total_wt += wt;
        
        if (tat > m.max_tat) m.max_tat = tat;
        if (wt > m.max_wt) m.max_wt = wt;
        
        completion_times.push_back(p.completion_time);
    }

    if (completed_procs_count > 0) {
        m.avg_tat = total_tat / completed_procs_count;
        m.avg_wt = total_wt / completed_procs_count;
    }

    for (size_t i = 0; i < THROUGHPUT_TIMES.size(); ++i) {
        int t = THROUGHPUT_TIMES[i];
        int count = 0;
        for (size_t j = 0; j < completion_times.size(); ++j) {
            if (completion_times[j] <= t) count++;
        }
        m.throughput[t] = count;
    }

    if (!gantt.empty()) {
        for (size_t i = 1; i < gantt.size(); ++i) {
            if (gantt[i].process_id != gantt[i-1].process_id && gantt[i-1].process_id != "IDLE") {
                m.context_switches++;
            }
        }
    }

    double last_time = gantt.empty() ? 0 : gantt.back().end;
    double busy_time = 0;
    for(size_t i = 0; i < gantt.size(); ++i) {
        if(gantt[i].process_id != "IDLE") {
            busy_time += (gantt[i].end - gantt[i].start);
        }
    }
    m.cpu_utilization = (last_time > 0) ? (busy_time / last_time) * 100.0 : 0.0;

    return m;
}

void write_results(string filename, string title, const Metrics& m, const vector<GanttSegment>& gantt) {
    ofstream out(filename.c_str());
    out << fixed << setprecision(3); // Tüm float deðerler 3 ondalýk hassasiyetle yazýlýr

    out << "--- SONUÇLAR: " << title << " ---\n\n";
    
    // a) ZAMAN TABLOSU
    out << "a) Zaman Tablosu\n";
    for (size_t i = 0; i < gantt.size(); ++i) {
        
        out << "[" << setw(7) << gantt[i].start << "] - - " << gantt[i].process_id << " - - [" << setw(7) << gantt[i].end << "]\n";
    }
    out << "[...] (Toplam " << gantt.size() << " segment)\n";
    out << "\n--------------------------------------------------\n";

    out << "b) Maksimum ve Ortalama Bekleme Süresi [Waitting Time]\n";
    out << "Maksimum Bekleme Süresi (Max WT): " << m.max_wt << "\n";
    out << "Ortalama Bekleme Süresi (Avg WT): " << m.avg_wt << "\n\n";

    out << "c) Maksimum ve Ortalama Tamamlanma Süresi [Turnaround Time]\n";
    out << "Maksimum Tamamlanma Süresi (Max TAT): " << m.max_tat << "\n";
    out << "Ortalama Tamamlanma Süresi (Avg TAT): " << m.avg_tat << "\n\n";

    out << "d) T=[50, 100, 150, 200] için Ýþ Tamamlama Sayýsý [Throughput]\n";
    for (map<int, int>::const_iterator it = m.throughput.begin(); it != m.throughput.end(); ++it) {
        out << "T=" << it->first << ": " << it->second << " süreç\n";
    }
    out << "\n";

    out << "e) Ortalama CPU Verimliliði [Baðlam Deðiþtirme Süresi: 0,001 birim zaman.]:\n";
    out << "CPU Verimliliði: " << m.cpu_utilization << "%\n\n";

    out << "f) Toplam Baðlam Deðiþtirme:\n";
    out << "Toplam Baðlam Deðiþtirme Sayýsý: " << m.context_switches << "\n";

    out.close();
    
    safe_print("Dosya olusturuldu: " + filename);
}

// --- ALGORÝTMA FONKSÝYONLARI ---

void schedule_non_preemptive(vector<Process> procs, int mode, string case_name, string algo_name) {
    sort(procs.begin(), procs.end(), [](const Process& a, const Process& b) {
        if (a.arrival_time != b.arrival_time) return a.arrival_time < b.arrival_time;
        return a.original_index < b.original_index;
    });

    vector<GanttSegment> gantt;
    double current_time = 0;
    int n = procs.size();
    string last_id = "NONE";

    while (true) {
        vector<int> candidates;
        for(int i=0; i<n; ++i) {
            if (!procs[i].is_completed && procs[i].arrival_time <= current_time) {
                candidates.push_back(i);
            }
        }

        if (candidates.empty()) {
            double next_arr = numeric_limits<double>::max();
            bool all_completed = true;
            for(int i=0; i<n; ++i) {
                if(!procs[i].is_completed) {
                    all_completed = false;
                    if (procs[i].arrival_time > current_time) {
                        next_arr = min(next_arr, procs[i].arrival_time);
                    }
                }
            }
            if(all_completed) break;
            if(next_arr != numeric_limits<double>::max()) {
                if(!gantt.empty() && gantt.back().process_id == "IDLE") gantt.back().end = next_arr;
                else gantt.push_back({current_time, next_arr, "IDLE"});
                current_time = next_arr;
                last_id = "IDLE";
            } else break; 
            continue;
        }

        sort(candidates.begin(), candidates.end(), [&](int i, int j) {
            if (mode == 1) { // SJF
                if (procs[i].burst_time != procs[j].burst_time) return procs[i].burst_time < procs[j].burst_time;
            } else if (mode == 2) { // Priority
                if (procs[i].priority != procs[j].priority) return procs[i].priority < procs[j].priority;
            }
            if (procs[i].arrival_time != procs[j].arrival_time) return procs[i].arrival_time < procs[j].arrival_time;
            return procs[i].original_index < procs[j].original_index;
        });

        int selected_idx = candidates[0];
        Process& p = procs[selected_idx];

        if (last_id != "NONE" && last_id != "IDLE") current_time += CONTEXT_SWITCH_TIME;

        double start = current_time;
        current_time += p.burst_time;
        p.start_time = (p.start_time == -1) ? start : p.start_time;
        p.completion_time = current_time;
        p.is_completed = true;
        gantt.push_back({start, current_time, p.id});
        last_id = p.id;
    }

    Metrics m = calculate_metrics(procs, gantt);
    write_results(case_name + "_" + algo_name + "_results.txt", case_name + " - " + algo_name, m, gantt);
}

void schedule_preemptive(vector<Process> procs, int mode, string case_name, string algo_name) {
    sort(procs.begin(), procs.end(), [](const Process& a, const Process& b) {
        if (a.arrival_time != b.arrival_time) return a.arrival_time < b.arrival_time;
        return a.original_index < b.original_index;
    });

    vector<GanttSegment> gantt;
    double current_time = 0;
    int n = procs.size();
    string last_id = "NONE";
    int running_idx = -1;

    while (true) {
        vector<int> ready_indices;
        for(int i=0; i<n; ++i) {
            if (!procs[i].is_completed && procs[i].arrival_time <= current_time) ready_indices.push_back(i);
        }

        if (ready_indices.empty()) {
            double next_arr = numeric_limits<double>::max();
            bool all_completed = true;
             for(int i=0; i<n; ++i) {
                if(!procs[i].is_completed) {
                    all_completed = false;
                    if (procs[i].arrival_time > current_time) next_arr = min(next_arr, procs[i].arrival_time);
                }
            }

            if(all_completed) break;
            
            if(next_arr != numeric_limits<double>::max()) {
                if(!gantt.empty() && gantt.back().process_id == "IDLE") gantt.back().end = next_arr;
                else gantt.push_back({current_time, next_arr, "IDLE"});
                current_time = next_arr;
                last_id = "IDLE";
            } else break;
            continue;
        }

        sort(ready_indices.begin(), ready_indices.end(), [&](int i, int j) {
            if (mode == 1) { // SJF
                if (abs(procs[i].remaining_time - procs[j].remaining_time) > 1e-9) return procs[i].remaining_time < procs[j].remaining_time;
            } else { // Priority
                if (procs[i].priority != procs[j].priority) return procs[i].priority < procs[j].priority;
            }
            return procs[i].arrival_time < procs[j].arrival_time;
        });

        int best_idx = ready_indices[0];

        if (running_idx != -1 && running_idx != best_idx) current_time += CONTEXT_SWITCH_TIME;

        running_idx = best_idx;
        Process& p = procs[best_idx];

        double next_completion = current_time + p.remaining_time;
        double next_arrival = numeric_limits<double>::max();
        for(int i=0; i<n; ++i) {
             if (!procs[i].is_completed && i != running_idx && procs[i].arrival_time > current_time) next_arrival = min(next_arrival, procs[i].arrival_time);
        }

        double next_event = min(next_completion, next_arrival);
        double time_slice = next_event - current_time;
        if(p.start_time == -1) p.start_time = current_time;
        gantt.push_back({current_time, current_time + time_slice, p.id});
        current_time += time_slice;
        p.remaining_time -= time_slice;
        last_id = p.id;

        if (p.remaining_time <= 1e-9) {
            p.remaining_time = 0;
            p.completion_time = current_time;
            p.is_completed = true;
            running_idx = -1;
        }
    }
    
    Metrics m = calculate_metrics(procs, gantt);
    write_results(case_name + "_" + algo_name + "_results.txt", case_name + " - " + algo_name, m, gantt);
}

void schedule_round_robin(vector<Process> procs, string case_name, string algo_name) {
    sort(procs.begin(), procs.end(), [](const Process& a, const Process& b) {
        if (a.arrival_time != b.arrival_time) return a.arrival_time < b.arrival_time;
        return a.original_index < b.original_index;
    });

    vector<GanttSegment> gantt;
    double current_time = 0;
    int n = procs.size();
    deque<int> ready_queue;
    vector<bool> in_queue(n, false);
    string last_id = "NONE";
    int proc_idx = 0; 

    auto push_arrived = [&]() {
        while(proc_idx < n && procs[proc_idx].arrival_time <= current_time) {
            if(!procs[proc_idx].is_completed && !in_queue[proc_idx]) {
                ready_queue.push_back(proc_idx);
                in_queue[proc_idx] = true;
            }
            proc_idx++;
        }
    };

    push_arrived();

    while(true) {
        if (ready_queue.empty()) {
            double next_arr = numeric_limits<double>::max();
            bool all_completed = true;
            for(int i=0; i<n; ++i) {
                if(!procs[i].is_completed) all_completed = false;
            }
            if(all_completed) break;

            if (proc_idx < n) next_arr = procs[proc_idx].arrival_time;
            if(next_arr != numeric_limits<double>::max()) {
                if(!gantt.empty() && gantt.back().process_id == "IDLE") gantt.back().end = next_arr;
                else gantt.push_back({current_time, next_arr, "IDLE"});
                current_time = next_arr;
                last_id = "IDLE";
                push_arrived();
            } else break;
            continue;
        }

        int idx = ready_queue.front();
        ready_queue.pop_front();
        in_queue[idx] = false;
        Process& p = procs[idx];

        if (last_id != "NONE" && last_id != "IDLE") current_time += CONTEXT_SWITCH_TIME;

        double time_to_run = min(p.remaining_time, RR_QUANTUM);
        double start = current_time;
        current_time += time_to_run;
        if(p.start_time == -1) p.start_time = start;
        p.remaining_time -= time_to_run;
        gantt.push_back({start, current_time, p.id});
        last_id = p.id;
        push_arrived();

        if (p.remaining_time <= 1e-9) {
            p.remaining_time = 0;
            p.completion_time = current_time;
            p.is_completed = true;
        } else {
            ready_queue.push_back(idx);
            in_queue[idx] = true;
        }
    }

    Metrics m = calculate_metrics(procs, gantt);
    write_results(case_name + "_" + algo_name + "_results.txt", case_name + " - " + algo_name, m, gantt);
}

// --- MAIN / VERÝ YÜKLEME ---

const string DATA_CASE_1 = R"(Process_ID,Arrival_Time,CPU_Burst_Time,Priority
P001,0,1,high
P002,2,2,normal
P003,4,3,low
P004,6,4,high
P005,8,5,normal
P006,10,6,low
P007,12,7,high
P008,14,8,normal
P009,16,9,low
P010,18,10,high
P011,20,11,normal
P012,22,12,low
P013,24,13,high
P014,26,14,normal
P015,28,15,low
P016,30,16,high
P017,32,17,normal
P018,34,18,low
P019,36,19,high
P020,38,20,normal
P021,40,1,low
P022,42,2,high
P023,44,3,normal
P024,46,4,low
P025,48,5,high
P026,50,6,normal
P027,52,7,low
P028,54,8,high
P029,56,9,normal
P030,58,10,low
P031,60,11,high
P032,62,12,normal
P033,64,13,low
P034,66,14,high
P035,68,15,normal
P036,70,16,low
P037,72,17,high
P038,74,18,normal
P039,76,19,low
P040,78,20,high
P041,80,1,normal
P042,82,2,low
P043,84,3,high
P044,86,4,normal
P045,88,5,low
P046,90,6,high
P047,92,7,normal
P048,94,8,low
P049,96,9,high
P050,98,10,normal
P051,100,11,low
P052,102,12,high
P053,104,13,normal
P054,106,14,low
P055,108,15,high
P056,110,16,normal
P057,112,17,low
P058,114,18,high
P059,116,19,normal
P060,118,20,low
P061,120,1,high
P062,122,2,normal
P063,124,3,low
P064,126,4,high
P065,128,5,normal
P066,130,6,low
P067,132,7,high
P068,134,8,normal
P069,136,9,low
P070,138,10,high
P071,140,11,normal
P072,142,12,low
P073,144,13,high
P074,146,14,normal
P075,148,15,low
P076,150,16,high
P077,152,17,normal
P078,154,18,low
P079,156,19,high
P080,158,20,normal
P081,160,1,low
P082,162,2,high
P083,164,3,normal
P084,166,4,low
P085,168,5,high
P086,170,6,normal
P087,172,7,low
P088,174,8,high
P089,176,9,normal
P090,178,10,low
P091,180,11,high
P092,182,12,normal
P093,184,13,low
P094,186,14,high
P095,188,15,normal
P096,190,16,low
P097,192,17,high
P098,194,18,normal
P099,196,19,low
P100,198,20,high
P101,200,1,normal
P102,202,2,low
P103,204,3,high
P104,206,4,normal
P105,208,5,low
P106,210,6,high
P107,212,7,normal
P108,214,8,low
P109,216,9,high
P110,218,10,normal
P111,220,11,low
P112,222,12,high
P113,224,13,normal
P114,226,14,low
P115,228,15,high
P116,230,16,normal
P117,232,17,low
P118,234,18,high
P119,236,19,normal
P120,238,20,low
P121,240,1,high
P122,242,2,normal
P123,244,3,low
P124,246,4,high
P125,248,5,normal
P126,250,6,low
P127,252,7,high
P128,254,8,normal
P129,256,9,low
P130,258,10,high
P131,260,11,normal
P132,262,12,low
P133,264,13,high
P134,266,14,normal
P135,268,15,low
P136,270,16,high
P137,272,17,normal
P138,274,18,low
P139,276,19,high
P140,278,20,normal
P141,280,1,low
P142,282,2,high
P143,284,3,normal
P144,286,4,low
P145,288,5,high
P146,290,6,normal
P147,292,7,low
P148,294,8,high
P149,296,9,normal
P150,298,10,low
P151,300,11,high
P152,302,12,normal
P153,304,13,low
P154,306,14,high
P155,308,15,normal
P156,310,16,low
P157,312,17,high
P158,314,18,normal
P159,316,19,low
P160,318,20,high
P161,320,1,normal
P162,322,2,low
P163,324,3,high
P164,326,4,normal
P165,328,5,low
P166,330,6,high
P167,332,7,normal
P168,334,8,low
P169,336,9,high
P170,338,10,normal
P171,340,11,low
P172,342,12,high
P173,344,13,normal
P174,346,14,low
P175,348,15,high
P176,350,16,normal
P177,352,17,low
P178,354,18,high
P179,356,19,normal
P180,358,20,low
P181,360,1,high
P182,362,2,normal
P183,364,3,low
P184,366,4,high
P185,368,5,normal
P186,370,6,low
P187,372,7,high
P188,374,8,normal
P189,376,9,low
P190,378,10,high
P191,380,11,normal
P192,382,12,low
P193,384,13,high
P194,386,14,normal
P195,388,15,low
P196,390,16,high
P197,392,17,normal
P198,394,18,low
P199,396,19,high
P200,398,20,normal)";

const string DATA_CASE_2 = R"(Process_ID,Arrival_Time,CPU_Burst_Time,Priority
P001,0,4,high
P002,2,7,normal
P003,4,10,low
P004,6,13,high
P005,8,16,normal
P006,10,19,low
P007,12,2,high
P008,14,5,normal
P009,16,8,low
P010,18,11,high
P011,20,14,normal
P012,22,17,low
P013,24,20,high
P014,26,3,normal
P015,28,6,low
P016,30,9,high
P017,32,12,normal
P018,34,15,low
P019,36,18,high
P020,38,1,normal
P021,40,4,low
P022,42,7,high
P023,44,10,normal
P024,46,13,low
P025,48,16,high
P026,50,19,normal
P027,52,2,low
P028,54,5,high
P029,56,8,normal
P030,58,11,low
P031,60,14,high
P032,62,17,normal
P033,64,20,low
P034,66,3,high
P035,68,6,normal
P036,70,9,low
P037,72,12,high
P038,74,15,normal
P039,76,18,low
P040,78,1,high
P041,80,4,normal
P042,82,7,low
P043,84,10,high
P044,86,13,normal
P045,88,16,low
P046,90,19,high
P047,92,2,normal
P048,94,5,low
P049,96,8,high
P050,98,11,normal
P051,100,14,low
P052,102,17,high
P053,104,20,normal
P054,106,3,low
P055,108,6,high
P056,110,9,normal
P057,112,12,low
P058,114,15,high
P059,116,18,normal
P060,118,1,low
P061,120,4,high
P062,122,7,normal
P063,124,10,low
P064,126,13,high
P065,128,16,normal
P066,130,19,low
P067,132,2,high
P068,134,5,normal
P069,136,8,low
P070,138,11,high
P071,140,14,normal
P072,142,17,low
P073,144,20,high
P074,146,3,normal
P075,148,6,low
P076,150,9,high
P077,152,12,normal
P078,154,15,low
P079,156,18,high
P080,158,1,normal
P081,160,4,low
P082,162,7,high
P083,164,10,normal
P084,166,13,low
P085,168,16,high
P086,170,19,normal
P087,172,2,low
P088,174,5,high
P089,176,8,normal
P090,178,11,low
P091,180,14,high
P092,182,17,normal
P093,184,20,low
P094,186,3,high
P095,188,6,normal
P096,190,9,low
P097,192,12,high
P098,194,15,normal
P099,196,18,low
P100,198,1,high)";

int main() {
    safe_print("Simulasyon baslatiliyor (C++ Multi-Threaded)...");

    map<string, vector<Process>> datasets;
    
    // Gömülü verilerden yükleme (Ödevdeki iki dosya için)
    datasets["Durum 1"] = load_processes_from_string(DATA_CASE_1);
    datasets["Durum 2"] = load_processes_from_string(DATA_CASE_2);

    // Harici dosya kullanmak isterseniz (esneklik koþulu için):
    // datasets["Yeni_Durum"] = load_processes_from_file("yeni_sürecler.csv");

    vector<thread> threads;

    for(map<string, vector<Process>>::const_iterator it = datasets.begin(); it != datasets.end(); ++it) {
        string case_name = it->first;
        vector<Process> procs = it->second;
        
        if (procs.empty()) {
            safe_print("UYARI: " + case_name + " icin islenecek surec bulunamadi. Atlandi.");
            continue;
        }

        //THREAD OLUÞTURMA
        threads.emplace_back(schedule_non_preemptive, procs, 0, case_name, "FCFS");
        threads.emplace_back(schedule_non_preemptive, procs, 1, case_name, "Non_Preemtive_SJF");
        threads.emplace_back(schedule_non_preemptive, procs, 2, case_name, "Non_Preemtive_Priority");
        
        threads.emplace_back(schedule_preemptive, procs, 1, case_name, "Preemtive_SJF");
        threads.emplace_back(schedule_preemptive, procs, 2, case_name, "Preemtive_Priority");
        
        threads.emplace_back(schedule_round_robin, procs, case_name, "Round_Robin");
    }

    // Tüm threadlerin bitmesini bekle (Join)
    for (size_t i = 0; i < threads.size(); ++i) {
        if (threads[i].joinable()) {
            threads[i].join();
        }
    }

    safe_print("\nTum simulasyonlar (Threadler) tamamlandi.");
    safe_print("Sonuclar .txt dosyalari olarak kaydedildi.");
    
    return 0;
}
