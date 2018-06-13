namespace generator {

void gen_points_in_traversable(EDBT::ObstacleMap* oMap, const vector<vector<pl::Point>>& polys,
                               int num, vector<pl::Point>& out) {
  long long min_x, max_x, min_y, max_y;
  // ignore border
  min_x = max_x = polys[1][0].x;
  min_y = max_y = polys[1][0].y;
  for (size_t i=1; i<polys.size(); i++) {
    for (const auto& p: polys[i]) {
      min_x = min(min_x, (long long)p.x);
      max_x = max(max_x, (long long)p.x);
      min_y = min(min_y, (long long)p.y);
      max_y = max(max_y, (long long)p.y);
    }
  }

  out.resize(num);
  random_device rd;
  mt19937 eng(rd());
  uniform_int_distribution<long long> distx(min_x, max_x);
  uniform_int_distribution<long long> disty(min_y, max_y);

  for (int i=0; i<num; i++) {
    long long x, y;
    do {
      x = distx(eng);
      y = disty(eng);
      pl::Point p{(double)x, (double)y};
      if (oMap->isCoveredByTraversable(p, p)) {
        out[i] = p;
        break;
      }
    } while (true);
  }
}
}// namespace generator
