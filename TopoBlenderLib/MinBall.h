// Synopsis: Library to find the smallest enclosing ball of points
//
// Authors: Martin Kutz <kutz@math.fu-berlin.de>,
//          Kaspar Fischer <kf@iaeth.ch>
#pragma once

#include <vector>

#define SEB_NAMESPACE Seb
#define SEB_ASSERT assert
#define SEB_ASSERT_EXPENSIVE(expr)
#define SEB_DEBUG(expr)
#define SEB_LOG(channel,expr)
#define SEB_TIMER_START(timer)
#define SEB_TIMER_PRINT(timer)
#define SEB_TIMER_STRING(timer)
#define SEB_STATS(expr)

// Warning about unsafe use of std::inner_product
#pragma warning( disable : 4996 ) 

namespace SEB_NAMESPACE {
  
  template<typename Float>
  class Point
  // A simple class representing a d-dimensional point.
  {
  public: // types:
    typedef typename std::vector<Float>::const_iterator Const_iterator;
    typedef typename std::vector<Float>::iterator Iterator;
    
  public: // construction and destruction:
    
    Point(int d)
    // Constructs a d-dimensional point with undefined coordinates.
    : c(d)
    {
    }
    
    template<typename InputIterator>
    Point(int d,InputIterator first)
    // Constructs a d-dimensional point with Cartesian center
    // coordinates [first,first+d).
    : c(first,first+d)
    {
    }
    
  public: // access:
    
    const Float& operator[](unsigned int i) const
    // Returns a const-reference to the i-th coordinate.
    {
      SEB_ASSERT(0 <= i && i < c.size());
      return c[i];
    }
    
    Float& operator[](unsigned int i)
    // Returns a reference to the i-th coordinate.
    {
      SEB_ASSERT(0 <= i && i < c.size());
      return c[i];
    }
    
    Const_iterator begin() const
    // Returns a const-iterator to the first of the d Cartesian coordinates.
    {
      return c.begin();
    }
    
    Const_iterator end() const
    // Returns the past-the-end iterator corresponding to begin().
    {
      return c.end();
    }
    
  private: // member fields:
    std::vector<Float> c;       // Cartesian center coordinates
  };
  
} // namespace SEB_NAMESPACE

namespace SEB_NAMESPACE {
  
  template<typename Float>
  inline Float sqr(const Float x)
  {
    return x * x;
  }
  
  template<typename Float, class Pt, class PointAccessor>
  class Subspan
  // An instance of this class represents the affine hull of a
  // non-empty set M of affinely independent points.  The set M is not
  // represented explicity; when an instance of this class is
  // constructed, you pass a list S of points to it (which the
  // instance will never change and which is assumed to stay fixed for
  // the lifetime of this instance): The set M is then a subset of S,
  // and its members are identified by their (zero-based) indices in
  // S.  The following routines are provided to query and change the
  // set M:
  //
  // - int size() returns the size of the instance's set M, a number
  //   between 0 and dim+1.
  //   Complexity: O(1).
  //
  // - bool is_member(int global_index) returns true iff S[global_index]
  //   is a member of M.
  //   Complexity: O(1)
  //
  // - global_index(int local_index) returns the index (into S) of the
  //   local_index-th point in M.  The points in M are internally
  //   ordered (in an arbitrary way) and this order only changes when
  //   add() or remove() (see below) is called.
  //   Complexity: O(1)
  //
  // - void add_point(int global_index) adds the global_index-th point
  //   of S to the instance's set M.
  //   Precondition: !is_member(global_index)
  //   Complexity: O(dim^2).
  //
  // - void remove_point(int local_index) removes the local_index-th
  //   point in M.
  //   Precondition: 0<=local_index<=size() and size()>1
  //   Complexity: O(dim^2)
  //
  // - int any_member() returns the global index (into S) of an
  //   arbitrary element of M.
  //   Precondition: size()>0
  //   Postcondition: is_member(any_member())
  //
  // The following routines are provided to query the affine hull of M:
  //
  // - void shortest_vector_to_span(p,w): Computes the vector w
  //   directed from point p to v, where v is the point in aff(M) that
  //   lies nearest to p.  Returned is the squared length of w.
  //   Precondition: size()>0
  //   Complexity: O(dim^2)
  //
  // - void find_affine_coefficients(c,coeffs):
  //   Preconditions: c lies in the affine hull aff(M) and size() > 0.
  //   Calculates the size()-many coefficients in the representation
  //   of c as an affine combination of the points M.  The i-th computed
  //   coefficient coeffs[i] corresponds to the i-th point in M, or,
  //   in other words, to the point in S with index global_index(i).
  //   Complexity: O(dim^2)
  {
  public: // construction and deletion:
    
    Subspan(unsigned int dim, const PointAccessor& S, int i);
    // Constructs an instance representing the affine hull aff(M) of M={p},
    // where p is the point S[i] from S.
    //
    // Notice that S must not changed as long as this instance of
    // Subspan<Float> is in use.
    
    ~Subspan();
    
  public: // modification:
    
    void add_point(int global_index);
    void remove_point(unsigned int local_index);
    
  public: // access:
    
    unsigned int size() const
    {
      return r+1;
    }
    
    bool is_member(unsigned int i) const
    {
      SEB_ASSERT(i < S.size());
      return membership[i];
    }
    
    unsigned int global_index(unsigned int i) const
    {
      SEB_ASSERT(i < size());
      return members[i];
    }
    
    unsigned int any_member() const {
      SEB_ASSERT(size()>0);
      return members[r];
    }
    
    template<typename RandomAccessIterator1,
    typename RandomAccessIterator2>
    Float shortest_vector_to_span(RandomAccessIterator1 p,
                                  RandomAccessIterator2 w);
    
    template<typename RandomAccessIterator1,
    typename RandomAccessIterator2>
    void find_affine_coefficients(RandomAccessIterator1 c,
                                  RandomAccessIterator2 coeffs);
    
  public: // debugging routines:
    
    Float representation_error();
    // Computes the coefficient representations of all points in the
    // (internally used) system (Q) and returns the maximal deviation
    // from the theoretical values.
    // Warning: This routine has running time O(dim^3).
    
  private: // private helper routines:
    
    void append_column();
    // Appends the new column u (which is a member of this instance) to
    // the right of "A = QR", updating Q and R.  It assumes r to still
    // be the old value, i.e., the index of the column used now for
    // insertion; r is not altered by this routine and should be changed
    // by the caller afterwards.
    // Precondition: r<dim
    
    void hessenberg_clear(unsigned int start);
    // Given R in lower Hessenberg form with subdiagonal entries 0 to
    // pos-1 already all zero, clears the remaining subdiagonal entries
    // via Givens rotations.
    
    void special_rank_1_update();
    // Update current QR-decomposition "A = QR" to
    // A + u [1,...,1] = Q' R'.
    
  private: // member fields:
    const PointAccessor &S;            // a const-reference to the set S
    std::vector<bool> membership;      // S[i] in M iff membership[i]
    const unsigned int dim;            // ambient dimension (not to be
    // confused with the rank r,
    // see below)
    
    // Entry i of members contains the index into S of the i-th point
    // in M.  The point members[r] is called the "origin."
    std::vector<unsigned int> members;
    
  private: // member fields for maintainin the QR-decomposition:
    Float **Q, **R;                    // (dim x dim)-matrices Q
    // (orthogonal) and R (upper
    // triangular); notice that
    // e.g.  Q[j][i] is the element
    // in row i and column j
    Float *u,*w;                       // needed for rank-1 update
    unsigned int r;                    // the rank of R (i.e. #points - 1)
  };
  
} // namespace SEB_NAMESPACE

// Synopsis: Class representing the affine hull of a point set.
//
// Authors: Martin Kutz <kutz@math.fu-berlin.de>,
//          Kaspar Fischer <kf@iaeth.ch>

#include <numeric>
#include <cmath>

// The point members[r] is called the origin; we use the following macro
// to increase the readibilty of the code:
#define SEB_AFFINE_ORIGIN S[members[r]]

namespace SEB_NAMESPACE {
  
  template<typename Float>
  inline void givens(Float& c, Float& s, const Float a, const Float b)
  //  Determine the Givens coefficients (c,s) satisfying
  //
  //     c * a + s * b = +/- (a^2 + b^2)
  //     c * b - s * a = 0
  //
  //  We don't care about the signs here for efficiency,
  //  so make sure not to rely on them anywhere.
  //
  //  Source: taken from "Matrix Computations" (2nd edition) by Gene
  //  H. B. Golub & Charles F. B. Van Loan (Johns Hopkins University
  //  Press, 1989), p. 216.
  {
    using std::abs;
    using std::sqrt;
    
    if (b == 0) {
      c = 1;
      s = 0;
    } else if (abs(b) > abs(a)) {
      const Float t = a / b;
      s = 1 / sqrt (1 + sqr(t));
      c = s * t;
    } else {
      const Float t = b / a;
      c = 1 / sqrt (1 + sqr(t));
      s = c * t;
    }
  }
  
  template<typename Float, class Pt, class PointAccessor>
  Subspan<Float, Pt, PointAccessor>::Subspan(unsigned int dim, const PointAccessor& S, int index)
  : S(S), membership(S.size()), dim(dim), members(dim+1)
  {
    // allocate storage for Q, R, u, and w:
    Q = new Float *[dim];
    R = new Float *[dim];
    for (unsigned int i=0; i<dim; ++i) {
      Q[i] = new Float[dim];
      R[i] = new Float[dim];
    }
    u = new Float[dim];
    w = new Float[dim];
    
    // initialize Q to the identity matrix:
    for (unsigned int i=0; i<dim; ++i)
      for (unsigned int j=0; j<dim; ++j)
        Q[i][j] = (i==j)? 1 : 0;
    
    members[r = 0] = index;
    membership[index] = true;
    
    SEB_LOG ("ranks",r << std::endl);
  }
  
  template<typename Float, class Pt, class PointAccessor>
  Subspan<Float, Pt, PointAccessor>::~Subspan()
  {
    for (unsigned int i=0; i<dim; ++i) {
      delete[] Q[i];
      delete[] R[i];
    }
    delete[] Q;
    delete[] R;
    delete[] u;
    delete[] w;
  }
  
  template<typename Float, class Pt, class PointAccessor>
  void Subspan<Float, Pt, PointAccessor>::add_point(int index) {
    SEB_ASSERT(!is_member(index));
    
    // compute S[i] - origin into u:
    for (unsigned int i=0; i<dim; ++i)
      u[i] = S[index][i] - SEB_AFFINE_ORIGIN[i];
    
    // appends new column u to R and updates QR-decomposition,
    // routine work with old r:
    append_column();
    
    // move origin index and insert new index:
    membership[index] = true;
    members[r+1] = members[r];
    members[r]   = index;
    ++r;
    
    SEB_LOG ("ranks",r << std::endl);
  }
  
  template<typename Float, class Pt, class PointAccessor>
  void Subspan<Float, Pt, PointAccessor>::remove_point(const unsigned int local_index) {
    SEB_ASSERT(is_member(global_index(local_index)) && size() > 1);
    
    membership[global_index(local_index)] = false;
    
    if (local_index == r) {
      // origin must be deleted
      
      // We choose the right-most member of Q, i.e., column r-1 of R,
      // as the new origin.  So all relative vectors (i.e., the
      // columns of "A = QR") have to be updated by u:= old origin -
      // S[global_index(r-1)]:
      for (unsigned int i=0; i<dim; ++i)
        u[i] = SEB_AFFINE_ORIGIN[i] - S[global_index(r-1)][i];
      
      --r;
      
      SEB_LOG ("ranks",r << std::endl);
      
      special_rank_1_update();
      
    } else {
      // general case: delete column from R
      
      //  shift higher columns of R one step to the left
      Float *dummy = R[local_index];
      for (unsigned int j = local_index+1; j < r; ++j) {
        R[j-1] = R[j];
        members[j-1] = members[j];
      }
      members[r-1] = members[r];  // shift down origin
      R[--r] = dummy;             // relink trash column
      
      // zero out subdiagonal entries in R
      hessenberg_clear(local_index);
    }
  }
  
  template<typename Float, class Pt, class PointAccessor>
  template<typename RandomAccessIterator1,
  typename RandomAccessIterator2>
  Float Subspan<Float, Pt, PointAccessor>::
  shortest_vector_to_span(RandomAccessIterator1 p,
                          RandomAccessIterator2 w)
  {
    using std::inner_product;
    
    // compute vector from p to origin, i.e., w = origin - p:
    for (unsigned int i=0; i<dim; ++i)
      w[i] = SEB_AFFINE_ORIGIN[i] - p[i];
    
    // remove projections of w onto the affine hull:
    for (unsigned int j = 0; j < r; ++j) {
      const Float scale = inner_product(w,w+dim,Q[j],Float(0));
      for (unsigned int i = 0; i < dim; ++i)
        w[i] -= scale * Q[j][i];
    }
    
    return inner_product(w,w+dim,w,Float(0));
  }
  
  template<typename Float, class Pt, class PointAccessor>
  Float Subspan<Float, Pt, PointAccessor>::representation_error()
  {
    using std::abs;
    
    std::vector<Float> lambdas(size());
    Float max = 0;
    Float error;
    
    // cycle through all points in hull
    for (unsigned int j = 0; j < size(); ++j) {
      // compute the affine representation:
      find_affine_coefficients(S[global_index(j)],lambdas.begin());
      
      // compare coefficient of point #j to 1.0
      error = abs(lambdas[j] - 1.0);
      if (error > max) max = error;
      
      // compare the other coefficients against 0.0
      for (unsigned int i = 0; i < j; ++i) {
        error = abs(lambdas[i] - 0.0);
        if (error > max) max = error;
      }
      for (unsigned int i = j+1; i < size(); ++i) {
        error = abs(lambdas[i] - 0.0);
        if (error > max) max = error;
      }
    }
    
    return max;
  }
  
  template<typename Float, class Pt, class PointAccessor>
  template<typename RandomAccessIterator1,
  typename RandomAccessIterator2>
  void Subspan<Float, Pt, PointAccessor>::
  find_affine_coefficients(RandomAccessIterator1 p,
                           RandomAccessIterator2 lambdas)
  {
    // compute relative position of p, i.e., u = p - origin:
    for (unsigned int i=0; i<dim; ++i)
      u[i] = p[i] - SEB_AFFINE_ORIGIN[i];
    
    // calculate Q^T u into w:
    for (unsigned int i = 0; i < dim; ++i) {
      w[i] = 0;
      for (unsigned int k = 0; k < dim; ++k)
        w[i] += Q[i][k] * u[k];
    }
    
    // We compute the coefficients by backsubstitution.  Notice that
    //
    //     c = \sum_{i\in M} \lambda_i (S[i] - origin)
    //       = \sum_{i\in M} \lambda_i S[i] + (1-s) origin
    //
    // where s = \sum_{i\in M} \lambda_i.-- We compute the coefficient
    // (1-s) of the origin in the variable origin_lambda:
    Float origin_lambda = 1;
    for (int j = r-1; j>=0; --j) {
      for (unsigned int k=j+1; k<r; ++k)
        w[j] -= *(lambdas+k) * R[k][j];
      origin_lambda -= *(lambdas+j) = w[j] / R[j][j];
    }
    // The r-th coefficient corresponds to the origin (cf. remove_point()):
    *(lambdas+r) = origin_lambda;
  }
  
  template<typename Float, class Pt, class PointAccessor>
  void Subspan<Float, Pt, PointAccessor>::append_column()
  // Appends the new column u (which is a member of this instance) to
  // the right of "A = QR", updating Q and R.  It assumes r to still
  // be the old value, i.e., the index of the column used now for
  // insertion; r is not altered by this routine and should be changed
  // by the caller afterwards.
  // Precondition: r<dim
  {
    SEB_ASSERT(r < dim);
    
    //  compute new column R[r] = Q^T * u
    for (unsigned int i = 0; i < dim; ++i) {
      R[r][i] = 0;
      for (unsigned int k = 0; k < dim; ++k)
        R[r][i] += Q[i][k] * u[k];
    }
    
    //  zero all entries R[r][dim-1] down to R[r][r+1]
    for (unsigned int j = dim-1; j > r; --j) {
      //  j is the index of the entry to be cleared
      //  with the help of entry j-1
      
      //  compute Givens coefficients c,s
      Float c, s;
      givens (c,s,R[r][j-1],R[r][j]);
      
      //  rotate one R-entry (the other one is an implicit zero)
      R[r][j-1] = c * R[r][j-1] + s * R[r][j];
      
      //  rotate two Q-columns
      for (unsigned int i = 0; i < dim; ++i) {
        const Float a = Q[j-1][i];
        const Float b = Q[j][i];
        Q[j-1][i] =  c * a + s * b;
        Q[j][i]   =  c * b - s * a;
      }
    }
  }
  
  template<typename Float, class Pt, class PointAccessor>
  void Subspan<Float, Pt, PointAccessor>::hessenberg_clear (unsigned int pos)
  // Given R in lower Hessenberg form with subdiagonal entries 0 to
  // pos-1 already all zero, clears the remaining subdiagonal entries
  // via Givens rotations.
  {
    //  clear new subdiagonal entries
    for (; pos < r; ++pos) {
      //  pos is the column index of the entry to be cleared
      
      //  compute Givens coefficients c,s
      Float c, s;
      givens (c,s,R[pos][pos],R[pos][pos+1]);
      
      //  rotate partial R-rows (of the first pair, only one entry is
      //  needed, the other one is an implicit zero)
      R[pos][pos] = c * R[pos][pos] + s * R[pos][pos+1];
      //  (then begin at posumn pos+1)
      for (unsigned int j = pos+1; j < r; ++j) {
        const Float a = R[j][pos];
        const Float b = R[j][pos+1];
        R[j][pos]   =  c * a + s * b;
        R[j][pos+1] =  c * b - s * a;
      }
      
      //  rotate Q-columns
      for (unsigned int i = 0; i < dim; ++i) {
        const Float a = Q[pos][i];
        const Float b = Q[pos+1][i];
        Q[pos][i]   =  c * a + s * b;
        Q[pos+1][i] =  c * b - s * a;
      }
    }
  }
  
  template<typename Float, class Pt, class PointAccessor>
  void Subspan<Float, Pt, PointAccessor>::special_rank_1_update ()
  // Update current QR-decomposition "A = QR" to
  // A + u * [1,...,1] = Q' R'.
  {
    //  compute w = Q^T * u
    for (unsigned int i = 0; i < dim; ++i) {
      w[i] = 0;
      for (unsigned int k = 0; k < dim; ++k)
        w[i] += Q[i][k] * u[k];
    }
    
    //  rotate w down to a multiple of the first unit vector;
    //  the operations have to be recorded in R and Q
    for (unsigned int k = dim-1; k > 0; --k) {
      //  k is the index of the entry to be cleared
      //  with the help of entry k-1
      
      //  compute Givens coefficients c,s
      Float c, s;
      givens (c,s,w[k-1],w[k]);
      
      //  rotate w-entry
      w[k-1] = c * w[k-1] + s * w[k];
      
      //  rotate two R-rows;
      //  the first column has to be treated separately
      //  in order to account for the implicit zero in R[k-1][k]
      R[k-1][k]    = -s * R[k-1][k-1];
      R[k-1][k-1] *=  c;
      for (unsigned int j = k; j < r; ++j) {
        const Float a = R[j][k-1];
        const Float b = R[j][k];
        R[j][k-1] =  c * a + s * b;
        R[j][k]   =  c * b - s * a;
      }
      
      //  rotate two Q-columns
      for (unsigned int i = 0; i < dim; ++i) {
        const Float a = Q[k-1][i];
        const Float b = Q[k][i];
        Q[k-1][i] =  c * a + s * b;
        Q[k][i]   =  c * b - s * a;
      }
    }
    
    //  add w * (1,...,1)^T to new R
    //  which means simply to add u[0] to each column
    //  since the other entries of u have just been eliminated
    for (unsigned int j = 0; j < r; ++j)
      R[j][0] += w[0];
    
    //  clear subdiagonal entries
    hessenberg_clear(0);
  }
  
} // namespace SEB_NAMESPACE



namespace SEB_NAMESPACE {

  // template arguments:
  // Float must be a floating point data type for which * / - + are defined
  // Pt[i] must return the i-th coordinate as a Float
  // PointAccessor[j] must return the j-th point in the data set as Pt and
  // size_t size() returns the size of the data set
  
  template<typename Float, class Pt = Point<Float>, class PointAccessor = std::vector<Pt> >
  class Smallest_enclosing_ball
  // An instance of class Smallest_enclosing_ball<Float> represents
  // the smallest enclosing ball of a set S of points.  Initially, the
  // set S is empty; you can add points by calling insert().
  {
  public: // iterator-type to iterate over the center coordinates of
	  // the miniball (cf. center_begin() below):
    typedef Float *Coordinate_iterator;
    
  public: // construction and destruction:

    Smallest_enclosing_ball(unsigned int d, PointAccessor &P)
    // Constructs an instance representing the miniball of points from
    // set S.  The dimension of the ambient space is fixed to d for
    // lifetime of the instance.
    : dim(d), S(P), up_to_date(true), support(NULL)
    {
      allocate_resources();
      SEB_ASSERT(!is_empty());
      update();
    }
    
    ~Smallest_enclosing_ball()
    {
      deallocate_resources();
    }
    
  public: // modification:
    
    void invalidate()
    // Notifies the instance that the underlying point set S (passed to the constructor
    // of this instance as parameter P) has changed. This will cause the miniball to
    // be recomputed lazily (i.e., when you call for example radius(), the recalculation
    // will be triggered).
    {
      up_to_date = false;
    }
    
  public: // access:
    
    bool is_empty()
    // Returns whether the miniball is empty, i.e., if no point has
    // been inserted so far.
    {
      return S.size() == 0;
    }
    
    Float squared_radius()
    // Returns the squared radius of the miniball.
    // Precondition: !is_empty()
    {
      if (!up_to_date)
        update();
      
      SEB_ASSERT(!is_empty());
      return radius_square;
    }
    
    Float radius()
    // Returns the radius of the miniball.
    // This is equivalent to std:sqrt(squared_radius())
    // Precondition: !is_empty()
    {
      if (!up_to_date)
        update();
      
      SEB_ASSERT(!is_empty());
      return radius_;
    }
    
    Coordinate_iterator center_begin()
    // Returns an iterator to the first Cartesian coordinate of the
    // center of the miniball.
    // Precondition: !is_empty()
    {
      if (!up_to_date)
        update();
      
      SEB_ASSERT(!is_empty());
      return center;
    }
    
    Coordinate_iterator center_end()
    // Returns the past-the-end iterator past the last Cartesian
    // coordinate of the center of the miniball.
    // Precondition: !is_empty
    {
      if (!up_to_date)
        update();
      
      SEB_ASSERT(!is_empty());
      return center+dim;
    }
    
  public: // testing:
    
    void verify();
    //  Verifies whether center lies really in affine hull,
    //  determines the consistency of the QR decomposition,
    //  and check whether all points of Q lie on the ball
    //  and all others within;
    //  prints the respective errors.
    
    void test_affine_stuff();
    // Runs some testing routines on the affine hull layer,
    // using the points in S.
    // Creates a new Q, so better use before or after actual computations.
    // Prints the accumulated error.
    
    
  private: // internal routines concerning storage:
    void allocate_resources();
    void deallocate_resources();
    
  private: // internal helper routines for the actual algorithm:
    void init_ball();
    Float find_stop_fraction(int& hinderer);
    bool successful_drop();
    
    void update();
    
  private: // we forbid copying (since we have dynamic storage):
    Smallest_enclosing_ball(const Smallest_enclosing_ball&);
    Smallest_enclosing_ball& operator=(const Smallest_enclosing_ball&);
    
  private: // member fields:
    unsigned int dim;                 // dimension of the amient space
    PointAccessor &S;                 // set S of inserted points
    bool up_to_date;                  // whether the miniball has
                                      // already been computed
    Float *center;                    // center of the miniball
    Float radius_, radius_square;     // squared radius of the miniball
    Subspan<Float, Pt, PointAccessor> *support;          // the points that lie on the current
    // boundary and "support" the ball;
    // the essential structure for update()
    
  private: // member fields for temporary use:
    Float *center_to_aff;
    Float *center_to_point;
    Float *lambdas;
    Float  dist_to_aff, dist_to_aff_square;
    
#ifdef SEB_STATS_MODE
  private: // memeber fields for statistics
    std::vector<int> entry_count;     // counts how often a point enters
    // the support; only available in
    // stats mode
#endif // SEB_STATS_MODE
    
  private: // constants:
    static const Float Eps;
  };
  
} // namespace SEB_NAMESPACE

// Synopsis: Library to find the smallest enclosing ball of points
//
// Authors: Martin Kutz <kutz@math.fu-berlin.de>,
//          Kaspar Fischer <kf@iaeth.ch>
#include <numeric>
#include <algorithm>

namespace SEB_NAMESPACE {
  
  template<typename Float, class Pt, class PointAccessor>
  void Smallest_enclosing_ball<Float, Pt, PointAccessor>::allocate_resources()
  {
    center            = new Float[dim];
    center_to_aff     = new Float[dim];
    center_to_point   = new Float[dim];
    lambdas           = new Float[dim+1];
  }
  
  template<typename Float, class Pt, class PointAccessor>
  void Smallest_enclosing_ball<Float, Pt, PointAccessor>::deallocate_resources()
  {
    delete[] center;
    delete[] center_to_aff;
    delete[] center_to_point;
    delete[] lambdas;
    
    if (support != NULL)
      delete support;
  }
  
  template<typename Float, class Pt, class PointAccessor>
  void Smallest_enclosing_ball<Float, Pt, PointAccessor>::init_ball()
  // Precondition: |S| > 0
  // Sets up the search ball with an arbitrary point of S as center
  // and with with exactly one of the points farthest from center in
  // support, which is instantiated here.
  // So the current ball contains all points of S and has radius at
  // most twice as large as the minball.
  {
    SEB_ASSERT(S.size() > 0);
    
    // set center to the first point in S:
    for (unsigned int i = 0; i < dim; ++i)
      center[i] = S[0][i];
    
    // find farthest point:
    radius_square = 0;
    unsigned int farthest = 0; // Note: assignment prevents compiler warnings.
    for (unsigned int j = 1; j < S.size(); ++j) {
      // compute squared distance from center to S[j]:
      Float dist = 0;
      for (unsigned int i = 0; i < dim; ++i)
        dist += sqr(S[j][i] - center[i]);
      
      // enlarge radius if needed:
      if (dist >= radius_square) {
        radius_square = dist;
        farthest = j;
      }
      radius_ = sqrt(radius_square);
    }
    
    // initialize support to the farthest point:
    if (support != NULL)
      delete support;
    support = new Subspan<Float, Pt, PointAccessor>(dim,S,farthest);
    
    // statistics:
    // initialize entry-counters to zero:
    SEB_STATS(entry_count = std::vector<int>(S.size(),0));
  }
  
  template<typename Float, class Pt, class PointAccessor>
  bool Smallest_enclosing_ball<Float, Pt, PointAccessor>::successful_drop()
  // Precondition: center lies in aff(support).
  // If center doesn't already lie in conv(support) and is thus
  // not optimal yet, successful_drop() elects a suitable point k to
  // be removed from support -- and returns true.
  // If center lies in the convex hull, however, false is returned
  // (and support remains unaltered).
  {
    // find coefficients of the affine combination of center:
    support->find_affine_coefficients(center,lambdas);
    
    // find a non-positive coefficient:
    unsigned int smallest = 0; // Note: assignment prevents compiler warnings.
    Float minimum(1);
    for (unsigned int i=0; i<support->size(); ++i)
      if (lambdas[i] < minimum) {
        minimum = lambdas[i];
        smallest = i;
      }
    
    // drop a point with non-positive coefficient, if any:
    if (minimum <= 0) {
      SEB_LOG ("debug","  removing local point #" << smallest << std::endl);
      support->remove_point(smallest);
      return true;
    }
    return false;
  }
  
  template<typename Float, class Pt, class PointAccessor>
  Float Smallest_enclosing_ball<Float, Pt, PointAccessor>::find_stop_fraction(int& stopper)
  // Given the center of the current enclosing ball and the
  // walking direction center_to_aff, determine how much we can walk
  // into this direction without losing a point from S.  The (positive)
  // factor by which we can walk along center_to_aff is returned.
  // Further, stopper is set to the index of the most restricting point
  // and to -1 if no such point was found.
  {
    using std::inner_product;
    
    // We would like to walk the full length of center_to_aff ...
    Float scale =  1;
    stopper     = -1;
    
    SEB_DEBUG (Float margin = 0;)
    
    // ... but one of the points in S might hinder us:
    for (unsigned int j = 0; j < S.size(); ++j)
      if (!support->is_member(j)) {
        
        // compute vector center_to_point from center to the point S[i]:
        for (unsigned int i = 0; i < dim; ++i)
          center_to_point[i] = S[j][i] - center[i];
        
        const Float dir_point_prod
        = inner_product(center_to_aff,center_to_aff+dim,
                        center_to_point,Float(0));
        
        // we can ignore points beyond support since they stay
        // enclosed anyway:
        if (dist_to_aff_square - dir_point_prod
            // make new variable 'radius_times_dist_to_aff'? !
            < Eps * radius_ * dist_to_aff)
          continue;
        
        // compute the fraction we can walk along center_to_aff until
        // we hit point S[i] on the boundary:
        // (Better don't try to understand this calculus from the code,
        //  it needs some pencil-and-paper work.)
        Float bound = radius_square;
        bound -= inner_product(center_to_point,center_to_point+dim,
                               center_to_point,Float(0));
        bound /= 2 * (dist_to_aff_square - dir_point_prod);
        
        // take the smallest fraction:
        if (bound < scale) {
          scale   = bound;
          stopper = j;
          SEB_DEBUG (margin = dist_to_aff - dir_point_prod / dist_to_aff;)
        }
      }
    
    SEB_LOG ("debug","  margin = " << margin << std::endl);
    
    return scale;
  }
  
  
  template<typename Float, class Pt, class PointAccessor>
  void Smallest_enclosing_ball<Float, Pt, PointAccessor>::update()
  // The main function containing the main loop.
  // Iteratively, we compute the point in support that is closest
  // to the current center and then walk towards this target as far
  // as we can, i.e., we move until some new point touches the
  // boundary of the ball and must thus be inserted into support.
  // In each of these two alternating phases, we always have to check
  // whether some point must be dropped from support, which is the
  // case when the center lies in aff(support).
  // If such an attempt to drop fails, we are done;  because then
  // the center lies even conv(support).
  {
    size_t iteration = 0;
    
    SEB_TIMER_START("computation");
    
    // optimistically, we set this flag now;
    // on return from this function it will be true:
    up_to_date = true;
    
    init_ball();
    
    // Invariant:  The ball B(center,radius_) always contains the whole
    // point set S and has the points in support on its boundary.

    // while (true) // <<== This was resulting in infinite loop in weird case..
    while (iteration < this->S.size() * 1000) 
	{
      ++iteration;
      SEB_LOG ("debug","  iteration " << iteration << std::endl);
      SEB_LOG ("debug","  " << support->size() << " points on boundary" << std::endl);
      
      // Compute a walking direction and walking vector,
      // and check if the former is perhaps too small:
      while ((dist_to_aff
              = sqrt(dist_to_aff_square
                     = support->shortest_vector_to_span(center,
                                                        center_to_aff)))
             <= Eps * radius_)
        // We are closer than Eps * radius_square, so we try a drop:
        if (!successful_drop()) {
          // If that is not possible, the center lies in the convex hull
          // and we are done.
          SEB_TIMER_PRINT("computation");
          return;
        }
      
      SEB_LOG ("debug","  distance to affine hull = "
               << dist_to_aff << std::endl);
      
      // determine how far we can walk in direction center_to_aff
      // without losing any point ('stopper', say) in S:
      int stopper;
      Float scale = find_stop_fraction(stopper);
      SEB_LOG ("debug","  stop fraction = " << scale << std::endl);
      
      if (stopper >= 0) {
        // stopping point exists
        
        // walk as far as we can
        for (unsigned int i = 0; i < dim; ++i)
          center[i] += scale * center_to_aff[i];
        
        // update the radius
        const Pt& stop_point = S[support->any_member()];
        radius_square = 0;
        for (unsigned int i = 0; i < dim; ++i)
          radius_square += sqr(stop_point[i] - center[i]);
        radius_ = sqrt(radius_square);
        SEB_LOG ("debug","  current radius = "
                 << std::setiosflags(std::ios::scientific)
                 << std::setprecision(17) << radius_
                 << std::endl << std::endl);
        
        // and add stopper to support
        support->add_point(stopper);
        SEB_STATS (++entry_count[stopper]);
        SEB_LOG ("debug","  adding global point #" << stopper << std::endl);
      }
      else {
        //  we can run unhindered into the affine hull
        SEB_LOG ("debug","  moving into affine hull" << std::endl);
        for (unsigned int i=0; i<dim; ++i)
          center[i] += center_to_aff[i];
        
        // update the radius:
        const Pt& stop_point = S[support->any_member()];
        radius_square = 0;
        for (unsigned int i = 0; i < dim; ++i)
          radius_square += sqr(stop_point[i] - center[i]);
        radius_ = sqrt(radius_square);
        SEB_LOG ("debug","  current radius = "
                 << std::setiosflags(std::ios::scientific)
                 << std::setprecision(17) << radius_
                 << std::endl << std::endl);
        
        // Theoretically, the distance to the affine hull is now zero
        // and we would thus drop a point in the next iteration.
        // For numerical stability, we don't rely on that to happen but
        // try to drop a point right now:
        if (!successful_drop()) {
          // Drop failed, so the center lies in conv(support) and is thus
          // optimal.
          SEB_TIMER_PRINT("computation");
          return;
        }
      }
    }
  }
  
  template<typename Float, class Pt, class PointAccessor>
  void Smallest_enclosing_ball<Float, Pt, PointAccessor>::verify()
  {
    using std::inner_product;
    using std::abs;
    using std::cout;
    using std::endl;
    
    Float  min_lambda      = 1;  // for center-in-convex-hull check
    Float  max_overlength  = 0;  // for all-points-in-ball check
    Float  min_underlength = 0;  // for all-boundary-points-on-boundary
    Float  ball_error;
    Float  qr_error = support->representation_error();
    
    // center really in convex hull?
    support->find_affine_coefficients(center,lambdas);
    for (unsigned int k = 0; k < support->size(); ++k)
      if (lambdas[k] <= min_lambda)
        min_lambda = lambdas[k];
    
    // all points in ball, all support points really on boundary?
    for (unsigned int k = 0; k < S.size(); ++k) {
      
      // compare center-to-point distance with radius
      for (unsigned int i = 0; i < dim; ++i)
        center_to_point[i] = S[k][i] - center[i];
      ball_error = sqrt(inner_product(center_to_point,center_to_point+dim,
                                      center_to_point,Float(0)))
      - radius_;
      
      // check for sphere violations
      if (ball_error > max_overlength) max_overlength = ball_error;
      
      // check for boundary violations
      if (support->is_member(k))
        if (ball_error < min_underlength) min_underlength = ball_error;
    }
    
    cout << "Solution errors (relative to radius, nonsquared)" << endl
    << "  final QR inconsistency     : " << qr_error << endl
    << "  minimal convex coefficient : ";
    if (min_lambda >= 0) cout << "positive";
    else cout << (-min_lambda);
    cout << endl
    << "  maximal overlength         : "
    << (max_overlength / radius_) << endl
    << "  maximal underlength        : "
    << (abs(min_underlength / radius_))
    << endl;
    
    
#ifdef SEB_STATS_MODE
    // And let's print some statistics about the rank changes
    cout << "=====================================================" << endl
    << "Statistics" << endl;
    
    // determine how often a single point entered support at most
    int max_enter = 0;
    for (int i = 0; i < S.size(); ++i)
      if (entry_count[i] > max_enter)
        max_enter = entry_count[i];
    ++max_enter;
    
    // compute a histogram from the entry data ...
    std::vector<int> histogram(max_enter+1);
    for (int j = 0; j <= max_enter; ++j)
      histogram[j] = 0;
    for (int i = 0; i < S.size(); ++i)
      histogram[entry_count[i]]++;
    // ... and print it
    for (int j = 0; j <= max_enter; j++)
      if (histogram[j])
        cout << histogram[j] << " points entered " << j << " times" << endl;
#endif // SEB_STATS MODE
  }
  
  
  template<typename Float, class Pt, class PointAccessor>
  void Smallest_enclosing_ball<Float, Pt, PointAccessor>::test_affine_stuff()
  {
    using std::cout;
    using std::endl;
    
    Float *direction;
    Float  error;
    Float  max_representation_error = 0;
    
    cout << S.size() << " points in " << dim << " dimensions" << endl;
    
    if (!is_empty()) {
      support = new Subspan<Float,PointAccessor,Pt>(dim,S,0);
      cout << "initializing affine subspace with point S[0]" << endl;
      
      direction = new Float[dim];
    }
    else return;
    
    for (int loop = 0; loop < 5; ++loop) {
      
      // Try to fill each point of S into aff
      for (int i = 0; i < S.size(); ++i) {
        
        cout << endl << "Trying new point #" << i << endl;
        
        Float dist = sqrt(support->shortest_vector_to_span(S[i],direction));
        cout << "dist(S[" << i << "],affine_hull) = "
        << dist << endl;
        
        if (dist > 1.0E-8) {
          cout << "inserting point S["<<i<<"] into affine hull"
          << endl;
          support->add_point(i);
          
          cout << "representation error: "
          << (error = support->representation_error()) << endl;
          if (error > max_representation_error)
            max_representation_error = error;
        }
      }
      
      //  Throw out half of the points
      while (support->size() > dim/2) {
        
        //  throw away the origin or point at position r/2,
        //  depending on whether size is odd or even:
        int k = support->size()/2;
        if (2 * k == support->size()) {
          //  size even
          cout << endl << "Throwing out local point #" << k << endl;
          support->remove_point(k);
        } else {
          //  size odd
          cout << endl << "Throughing out the origin" << endl;
          support->remove(support->size()-1);
        }
        
        cout << "representation error: "
        << (error = support->representation_error()) << endl;
        if (error > max_representation_error)
          max_representation_error = error;
      }
    }
    
    cout << "maximal representation error: "
    << max_representation_error << endl
    << "End of test." << endl;
    
    delete support;
    delete direction;
  }
  
  
  template<typename Float, class Pt, class PointAccessor>
  const Float Smallest_enclosing_ball<Float, Pt, PointAccessor>::Eps = Float(1e-14);
  
} // namespace SEB_NAMESPACE

typedef Seb::Smallest_enclosing_ball<double> MinBall;

// Converter
template<typename Float, typename Vector3>
static inline std::vector< Seb::Point<Float> > SebPoints( std::vector< Vector3 > points )
{
    std::vector< Seb::Point<Float> > pnts;

    for (int i = 0; i < (int) points.size(); ++i)
    {
		std::vector<Float> pvector(3, 0);
		pvector[0] = points[i][0];
		pvector[1] = points[i][1];
		pvector[2] = points[i][2];
        pnts.push_back(Seb::Point<Float>(3, pvector.begin()));
    }

    return pnts;
}

template<typename Vector3>
static void MinBallCenter(MinBall & mb, Vector3 & center)
{
	MinBall::Coordinate_iterator center_it = mb.center_begin();
	center[0] = center_it[0];
	center[1] = center_it[1];
	center[2] = center_it[2];
}

// Assuming Vector3 is a container with 3 scalars, see above
template<typename Vector3>
struct QuickMinBall{
	QuickMinBall(std::vector<Vector3> pnts){
		std::vector< Seb::Point<double> > sebPoints = SebPoints<double,Vector3>(pnts);
		MinBall mb(3, sebPoints);
		radius = mb.radius();
		MinBallCenter<Vector3>(mb, center);
	}
	Vector3 center;
	double radius;
};
