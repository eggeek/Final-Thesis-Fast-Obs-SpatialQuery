namespace pl = polyanya;
namespace rs = rstar;
namespace generator {

bool is_self_intersected(const vector<pl::Point>& poly) {
  int N = (int)poly.size();
  if (N == 3) return false;
  for (int i=0; i<N; i++) {
    pl::Point v0 = poly[i];
    pl::Point v1 = poly[(i+1) % N];
    for (int j=i+2; j<N; j++) {
      pl::Point u0 = poly[j];
      pl::Point u1 = poly[(j+1) % N];
      if ((j+1) % N == i) continue;
      if (is_intersect(v0, v1, u0, u1))
        return true;
    }
  }
  return false;
}

void simplify_polys(const vector<vector<pl::Point>>& polys, vector<vector<pl::Point>>& simplified) {
  for (const auto& poly: polys) {
    vector<pl::Point> simpPoly;
    for (const auto p: poly) {
      int sz = simpPoly.size();
      if (simpPoly.empty() || p.distance(simpPoly.back()) > EPSILON) {
        if (sz >= 2 && is_collinear(simpPoly[sz-2], simpPoly[sz-1], p))
          simpPoly.pop_back();
        simpPoly.push_back(p);
      }
    }
    if (simpPoly.back().distance(simpPoly.front()) <= EPSILON)
      simpPoly.pop_back();
      simplified.push_back(simpPoly);
    }
  }
}

rs::Mbr getPolyMbr(const vector<pl::Point>& poly) {
  double min_x, min_y, max_x, max_y;
  min_x = min_y = INF;
  max_x = max_y = -INF;
  for (const auto& it: poly) {
    min_x = min(min_x, it.x);
    min_y = min(min_y, it.y);
    max_x = max(max_x, it.x);
    max_y = max(max_y, it.y);
  }
  return rs::Mbr(min_x, max_x, min_y, max_y);
}

void fit_to_box(vector<pl::Point>& poly, double cx, double cy, double len) {
  // top-left corner: (cx, cy)
  // size of box: len
  rs::Mbr mbr = getPolyMbr(poly);
  double min_x = mbr.coord[0][0], min_y = mbr.coord[1][0];
  double max_x = mbr.coord[0][1], max_y = mbr.coord[1][1];
  // move to (0, 0)
  for (auto& p: poly) {
    p.x -= min_x;
    p.y -= min_y;
  }
  // normalize
  for (auto& p: poly) {
    p.x = floor(p.x / (max_x - min_x) * len);
    p.y = floor(p.y / (max_y - min_y) * len);
  }
  rs::Mbr mbr2 = getPolyMbr(poly);
  // move top-lef corner to (cx, cy)
  for (auto& p: poly) {
    p.x += cx;
    p.y += cy;
    assert(p.x >= cx - EPSILON && p.x <= cx + len + EPSILON);
    assert(p.y >= cy - EPSILON && p.y <= cy + len + EPSILON);
  }
}

void sort_by_corner(vector<vector<pl::Point>>& polys) {
  auto cmp = [&](vector<pl::Point>& a, vector<pl::Point>& b) {
    double min_xa, min_xb, min_ya, min_yb;
    rs::Mbr mbra = getPolyMbr(a);
    rs::Mbr mbrb = getPolyMbr(b);
    min_xa = mbra.coord[0][0], min_ya = mbra.coord[1][0];
    min_xb = mbrb.coord[0][0], min_yb = mbrb.coord[1][0];
    if (fabs(min_ya - min_yb) > EPSILON)
      return min_ya < min_yb;
    else
      return min_xa < min_xb;
  };
  sort(polys.begin(), polys.end(), cmp);
}

void normalize_polys(vector<vector<pl::Point>>& polys,  double size) {
  sort_by_corner(polys);
  int tot = (int)polys.size();
  int num = sqrt((double)tot) + 1;
  double len = floor(size / ((double)num + 2.0));
  double d = 10;

  int cur = 0;
  for (int i=1; i<=num && cur < tot; i++) {
    for (int j=1; j<=num && cur < tot; j++) {
      double cx = (double)i * len;
      double cy = (double)j * len;
      fit_to_box(polys[cur++], cx + d, cy + d, (len - d) * 0.9);
      rs::Mbr mbr = getPolyMbr(polys[cur-1]);
    }
  }
  // add border
  polys.insert(polys.begin(), {{d, d}, {size-d, d}, {size-d, size-d}, {d, size-d}});
}

void random_choose_polys(vector<vector<pl::Point>>& polys, int num) {
  std::random_shuffle(polys.begin(), polys.end());
  if (num < (int)polys.size())
    polys.erase(polys.begin() + num, polys.end());
}

vector<vector<pl::Point>> remove_bad_polys(vector<vector<pl::Point>>& raw_polys) {
  vector<vector<pl::Point>> res;
  for (auto& it: raw_polys) {
    if (!is_self_intersected(it))
      res.push_back(it);
  }
  return res;
}
}// namesapce geneartor
