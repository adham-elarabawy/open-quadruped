class InverseKinematics {
  private:
  double hip_offset[2];
  double wrist;
  double shoulder;

  public:
  void init(double hip_offset_0, double hip_offset_1, double temp_shoulder, double temp_wrist);
  double * run(int legIndex, double x, double y, double z);
};
