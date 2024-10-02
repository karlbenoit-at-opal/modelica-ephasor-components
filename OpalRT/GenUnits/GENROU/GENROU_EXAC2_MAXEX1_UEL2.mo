within OpalRT.GenUnits.GENROU;
class GENROU_EXAC2_MAXEX1_UEL2
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // EXAC2 Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real TB_ex = 12 "(sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real TC_ex = 10 "(sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KA_ex = 400 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real TA_ex = 5 "(sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real VAMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real VAMIN_ex = -5 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KB_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KL_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KH_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real TF_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KD_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real VLR_ex = 4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "EXAC2 Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "EXAC2 Parameters"));
  //
  // UEL2 parameters
  parameter Real TUV_uel = 5 "(sec) voltage filter time constant" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TUP_uel = 5 "(sec) real power filter time constant" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TUQ_uel = 0 "(sec) reactive power filter time constant" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real KUI_uel = 0.5 "(pu) UEL integral gain" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real KUL_uel = 0.8 "(pu) UEL proportional gain" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real VUIMAX_uel = 0.25 "(pu) UEL integrator output maximum limit" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real VUIMIN_uel = 0 "(pu) UEL integrator output minimum limit" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real KUF_uel = 0 "(pu) UEL excitation system stabilizer gain" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real KFB_uel = 0 "(pu)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TUL_uel = 0 "(sec)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TU1_uel = 0 "UEL lead time constant (sec)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TU2_uel = 0 "UEL lag time constant (sec)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TU3_uel = 0 "UEL lead time constant (sec)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TU4_uel = 0 "UEL lag time constant (sec)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P0_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q0_uel = -0.31 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P1_uel = 0.3 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q1_uel = -0.31 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P2_uel = 0.6 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q2_uel = -0.28 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P3_uel = 0.9 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q3_uel = -0.21 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P4_uel = 1.02 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q4_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P5_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q5_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P6_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q6_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P7_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q7_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P8_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q8_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P9_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q9_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P10_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q10_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real VULMAX_uel = 0.25 "(pu) UEL output maximum limit" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real VULMIN_uel = 0 "(pu) UEL output minimum limit" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real M0_uel = 0 "k1, exponent in function F1" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real M1_uel = 0 "k2, exponent in function F2" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real M2_uel = 0 " 0: MVAR curve interpreted as mirror image around MVAR axis; 1: MVAR is found by linear extrapolation" annotation(Dialog(tab = "UEL2 Parameters"));
  // MAXEX1 Parameters
  parameter Real EFDRATED_OEL = 2.8 annotation(Dialog(tab = "MAXEX1 Parameters"));
  parameter Real EFD1_OEL = 1.2 annotation(Dialog(tab = "MAXEX1 Parameters"));
  parameter Real TIME1_OEL = 50 annotation(Dialog(tab = "MAXEX1 Parameters"));
  parameter Real EFD2_OEL = 1.3 annotation(Dialog(tab = "MAXEX1 Parameters"));
  parameter Real TIME2_OEL = 30 annotation(Dialog(tab = "MAXEX1 Parameters"));
  parameter Real EFD3_OEL = 1.5 annotation(Dialog(tab = "MAXEX1 Parameters"));
  parameter Real TIME3_OEL = 5 annotation(Dialog(tab = "MAXEX1 Parameters"));
  parameter Real EFDDES_OEL = 1 annotation(Dialog(tab = "MAXEX1 Parameters"));
  parameter Real KMX_OEL = 1.5 annotation(Dialog(tab = "MAXEX1 Parameters"));
  parameter Real VLOW_OEL = -0.1 annotation(Dialog(tab = "MAXEX1 Parameters"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin={19,-15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin={64,-40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXAC2 exac21(ID = M_ID, TR = TR_ex, TB = TB_ex, TC = TC_ex, KA = KA_ex, TA = TA_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, KB = KB_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, TE = TE_ex, KL = KL_ex, KH = KH_ex, KF = KF_ex, TF = TF_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, VLR = VLR_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-23,-1}, extent={{-17,-17},
            {17,17}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={14,10}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin={-96,40},   extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-60,-16}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.OverExcitationLimiter.MAXEX1 maxex11(EFDRATED = EFDRATED_OEL, EFD1 = EFD1_OEL, TIME1 = TIME1_OEL, EFD2 = EFD2_OEL, TIME2 = TIME2_OEL, EFD3 = EFD3_OEL, TIME3 = TIME3_OEL, EFDDES = EFDDES_OEL, KMX = KMX_OEL, VLOW = VLOW_OEL) annotation(Placement(visible = true, transformation(origin={-70,4}, extent={{-10,-10},
            {10,10}}, rotation = 0)));
  OpalRT.Electrical.Control.UnderExcitationLimiter.UEL2 uel21(TUV = TUV_uel, TUP = TUP_uel, TUQ = TUQ_uel, KUI = KUI_uel, KUL = KUL_uel, VUIMAX = VUIMAX_uel, VUIMIN = VUIMIN_uel, KUF = KUF_uel, KFB = KFB_uel, TUL = TUL_uel, TU1 = TU1_uel, TU2 = TU2_uel, TU3 = TU3_uel, TU4 = TU4_uel, P0 = P0_uel, Q0 = Q0_uel, P1 = P1_uel, Q1 = Q1_uel, P2 = P2_uel, Q2 = Q2_uel, P3 = P3_uel, Q3 = Q3_uel, P4 = P4_uel, Q4 = Q4_uel, P5 = P5_uel, Q5 = Q5_uel, P6 = P6_uel, Q6 = Q6_uel, P7 = P7_uel, Q7 = Q7_uel, P8 = P8_uel, Q8 = Q8_uel, P9 = P9_uel, Q9 = Q9_uel, P10 = P10_uel, Q10 = Q10_uel, VULMAX = VULMAX_uel, VULMIN = VULMIN_uel, M0 = M0_uel, M1 = M1_uel, M2 = M2_uel) annotation(Placement(visible = true, transformation(origin={-78,34}, extent={{-10,-10},
            {25,10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-58,-42}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin={-96,-40},  extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation 
  connect(genrou1.p, bus0) annotation(Line(points = {{19,-30},{19.6701,-30},{
          19.6701,-40.6963},{64,-40.6963},{64,-40}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{14,10},{18.9166,10},{
          18.9166,0},{19,0}}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{4,-27},{2,
          -27},{2,-24},{4,-24}}, color = {0,0,127}));
  connect(exac21.EFD0, genrou1.EFD0) annotation (Line(points = {{-6,-14.6},{-3,
          -14.6},{-3,-15},{3.7,-15}}, color = {0,0,127}));
  connect(exac21.EFD, genrou1.EFD) annotation (Line(points = {{-6,-11.2},{-4,
          -11.2},{-4,-10.8},{4,-10.8}}, color = {0,0,127}));
  connect(exac21.ETERM0, genrou1.ETERM0) annotation (Line(points = {{-6,-7.8},{
          -4,-7.8},{-4,-7.5},{3.7,-7.5}}, color = {0,0,127}));
  connect(exac21.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{-6,-3.38},
          {-3,-3.38},{-3,-3},{3.7,-3}}, color = {0,0,127}));
  connect(exac21.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-40,7.5},{
          -46,7.5},{-46,24},{40,24},{40,-7.5},{34,-7.5}}, color = {0,0,127}));
  connect(const.y, exac21.VOTHSG) annotation (Line(points = {{-54.5,-16},{-52,
          -16},{-52,-7.12},{-40,-7.12}}, color = {0,0,127}));
  connect(maxex11.VOEL, exac21.VOEL) annotation (Line(points = {{-59.6,4},{-48,
          4},{-48,-2.36},{-40,-2.36}}, color = {0,0,127}));
  connect(exac21.VUEL, uel21.VUEL) annotation (Line(points={{-40,2.4},{-50,2.4},
          {-50,34},{-52.3,34}}, color = {0,0,127}));
  connect(uel21.VF, exac21.VF) annotation (Line(points={{-87.65,34},{-92,34},{-92,
          60},{0,60},{0,12.6},{-5.32,12.6}},      color = {0,0,127}));
  connect(uel21.EX_AUX, genrou1.EX_AUX) annotation (Line(points={{-87.65,42},{-96,
          42},{-96,62},{2,62},{2,-4},{3.7,-4},{3.7,-3}},
        color = {0,0,127}));
  connect(maxex11.EFD, genrou1.EFD) annotation (Line(points = {{-79.8,10},{-86,
          10},{-86,-32},{0,-32},{0,-10.8},{4,-10.8}},
                                                  color = {0,0,127}));
  connect(maxex11.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-79.8,-2},
          {-84,-2},{-84,16},{-46,16},{-46,24},{40,24},{40,-7.5},{34,-7.5}},
        color = {0,0,127}));
  connect(dVREF, exac21.dVREF) annotation (Line(points={{-58,-42},{-46,-42},{-46,
          -11.2},{-40,-11.2}}, color={0,0,0}));
  connect(genrou1.VI, exac21.VI) annotation (Line(points={{34,-15},{48,-15},{48,
          20},{6,20},{6,9.2},{-6,9.2}}, color={0,0,127}));
  connect(genrou1.VI, uel21.VI) annotation (Line(points={{34,-15},{48,-15},{48,50},
          {-92,50},{-92,38},{-87.65,38}}, color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {1.13895, -0.455581}, extent = {{-95.8998, 87.016}, {94.0775, -92.2551}}), Text(origin = {57.0615, -57.2869}, extent = {{25.85, -23.58}, {-20.1552, 17.8852}}, textString = "PIN"), Text(origin = {-5.13, 9.23}, extent = {{-73.46, 18.79}, {83.94, -29.04}}, textString = "GENROU_EXAC2_MAXEX1_UEL2")}));
end GENROU_EXAC2_MAXEX1_UEL2;
