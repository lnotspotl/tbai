void testCppADCG_dynamics_flow_map_jacobian_sparsity(unsigned long const** row,
                                                     unsigned long const** col,
                                                     unsigned long* nnz) {
   static unsigned long const rows[24] = {0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,3,3};
   static unsigned long const cols[24] = {1,2,3,4,5,6,1,2,3,4,5,6,1,2,3,4,5,6,1,2,3,4,5,6};
   *row = rows;
   *col = cols;
   *nnz = 24;
}
