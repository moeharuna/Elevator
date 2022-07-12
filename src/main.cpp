#include <vector>
#include <map>
#include <cassert>
#include <algorithm>
#include <iostream>

class Direction {
public:
  enum Value {
    UP  = 1,
    DOWN = -1
  };
  Direction() = default;
  Direction(Value d) : value(d){};
  Direction(const int start, const int finish) {
    assert(start != finish);
    if(start < finish) value = Direction::UP;
    else value = Direction::DOWN;
  }

  operator Value() const { return value;}
  explicit operator bool() const  =delete;

  Direction operator!() const {
    if(value==UP) return DOWN;
    return UP;
  }
private:
  Value value;
};

class Floors {
  public:
  Floors(std::vector<std::vector<int>> floors):floors_{floors} {
  }

  const size_t size() const {
    return floors_.size();
  }

  const size_t people_count() const {
    int size =0;
    for(const auto& floor: floors_) {
      size+=floor.size();
    }
    return size;
  }

  const std::vector<int>& operator[](int floor) const {
    assert(floor < floors_.size());
    return floors_[floor];
  }

  std::vector<int>& operator[](int floor) {
    assert(floor < floors_.size());
    return floors_[floor];
  }
private:
  std::vector<std::vector<int>> floors_;
};


class Elevator {
private:
  void add_person(const int i) {
    ppl_inside_[i]++;
    inside_count_++;
  }

  void people_enter() {
    bool apply_empty_rule = false;
    auto& current_floor = floors_[position_];
    if(current_floor.size()==0) return;

    bool all_same = true;
    Direction target_direction = Direction(position_, current_floor[0]);

    for(int i=0; i<current_floor.size(); ++i) {
      all_same = all_same && Direction(position_, current_floor[i]) == target_direction;
    }

    if(inside_count_==0 && all_same)
      {
        apply_empty_rule = true;
      }

    for(int i=0;i < current_floor.size() && inside_count_ < capacity_;){ //current_floor.size() should be diffrent after remove_person, so should be safe.

      if(apply_empty_rule || Direction(position_, current_floor[i])==direction_) {
        add_person(current_floor[i]);
        current_floor.erase(current_floor.begin()+i);
      } else {
        ++i;
      }
    }
  }

  void people_exit() {
    inside_count_ -= ppl_inside_[position_];
    ppl_inside_[position_] = 0;
  }

  void open_doors() {
    people_exit();
    people_enter();
    assert(position_ < floors_.size());
    stop_history_.emplace_back(position_);
  }

  bool on_edge(int floor) {
    return
      (floor == floors_.size()-1 && direction_ == Direction::UP) ||
      (floor == 0                && direction_ == Direction::DOWN);
  }

  bool should_stop(int floor) {
    const auto& current_floor = floors_[floor];
    if(on_edge(position_)) {
        return true;
    }
    bool someone_wants_in = false;
    for(int i=0; i<current_floor.size(); ++i) {
      someone_wants_in = someone_wants_in || Direction(floor, current_floor[i])==direction_;
    }
    return ppl_inside_[floor] >0 || someone_wants_in;
  }

public:
  Elevator(int capacity, const std::vector<std::vector<int>> &queue):
    floors_{Floors{queue}},
    capacity_{capacity}
  {
    for(int i=0; i<floors_.size(); ++i) {
      ppl_inside_[i] =0;
    }
  }

  void transfer_all() {
    while(floors_.people_count() > 0 || inside_count_ >0) {
      if(should_stop(position_))  {
        if(on_edge(position_)) {
          direction_ = !direction_;
        }
        open_doors();
      }

      position_+=direction_;
    }
  }

  const std::vector<int>& floor_history() const{
    return stop_history_;
  }

private:
  Floors floors_;
  const int capacity_;
  int inside_count_ = 0;
  Direction direction_ = Direction::UP;
  int position_ = 0;
  std::map<int, int> ppl_inside_;
  std::vector<int> stop_history_;
};

std::vector<int> perform_caclculations(int capacity, std::vector<std::vector<int>> &&queue) {
  Elevator elevator{capacity, std::move(queue)};
  elevator.transfer_all();
  auto result = elevator.floor_history();
  return result;
}

void print_vector(std::vector<int> vec) {
  std::cout<<"{";
  for(int i=0; i<vec.size(); ++i) {
    std::cout<<vec[i]<<" ";
  }
  std::cout<<"}\n";
}

int main(int argc, char**argv) {
  int capacity = 5;

  std::vector<std::vector<int>> queue {
    {},
    {0, 2, 2, 0, 3},
    {},
    {0, 0, 0},
    {2},
    {3, 1, 1, 0},
  };
  //floorsList = {1, 2, 3, 5, 4, 3, 1, 0, 5, 4, 2, 1, 0}
  auto result =perform_caclculations(capacity, std::move(queue));
  print_vector(result);
  std::vector<int> check = {1, 2, 3, 5, 4, 3, 1, 0, 5, 4, 2, 1, 0};
  assert(result == check);

  return 0;
}
