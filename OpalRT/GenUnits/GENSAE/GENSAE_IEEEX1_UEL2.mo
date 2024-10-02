within OpalRT.GenUnits.GENSAE;
class GENSAE_IEEEX1_UEL2
  parameter Real partType = 1;
  // GENSAE Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Tdo_p = 10 "d-axis transient time constant" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Tdo_s = 0.05 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Tqo_s = 0.01 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENSAE Parameters"));
  // IEEEX1 Parameters
  parameter Real TR_ex = 0.025 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real KA_ex = 98 annotation(Dialog(tab = "IEEEX1"));
  parameter Real TA_ex = 0.2 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real TB_ex = 0.5 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real TC_ex = 1 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real VRMAX_ex = 9 "or zero" annotation(Dialog(tab = "IEEEX1"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "IEEEX1"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "IEEEX1"));
  parameter Real TE_ex = 0.35 "(>0) (sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real KF_ex = 0.03 annotation(Dialog(tab = "IEEEX1"));
  parameter Real TF1_ex = 0.4 "(>0) (sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real Switch_ex = 0 annotation(Dialog(tab = "IEEEX1"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "IEEEX1"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "IEEEX1"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "IEEEX1"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "IEEEX1"));
  // HYGOV Parameters
  parameter Real R_tg = 0.06 "Permanent Droop" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real r_tg = 0.4 "Temporary Droop" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tr_tg = 8 "(>0) Governor time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tf_tg = 0.05 "(>0) Filter time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tg_tg = 0.2 "(>0) Servo time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real VELM_tg = 0.01 "Gate velocity limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real GMAX_tg = 0.601 "Maximum gate limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real GMIN_tg = 0 "Minimum gate limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real TW_tg = 1.2 "(>0) Water time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real At_tg = 2.5 "Trubine gain" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Dturb_tg = 0 "Turbine damping" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real qNL_tg = 0.5 "No power flow" annotation(Dialog(tab = "HYGOV Parameters"));
  //
  // IEEEST Parameters
  parameter Real A1_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A2_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A3_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A4_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A5_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A6_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T1_pss = 0.03 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T2_pss = 0.01 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T3_pss = 0.02 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T4_pss = 0.01 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T5_pss = 0.2 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T6_pss = 0.1 "(>0)(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real KS_pss = -5 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMAX_pss = 6 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMIN_pss = -6 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCU_pss = 1 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCL_pss = 1 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  // IEEEST ICONs
  parameter Real M0_pss = 1 "Stabilizer input code" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  parameter Real M1_pss = 1 "IB, remote bus number" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
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
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin={56,-18}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEEX1 ieeex11(TR = TR_ex, KA = KA_ex, TA = TA_ex, TB = TB_ex, TC = TC_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={14,8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAE gensae1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-6.5,5.5}, extent={{-2.5,
            -2.5},{2.5,2.5}}, rotation = 0)));
  OpalRT.Electrical.Control.UnderExcitationLimiter.UEL2 uel21(TUV = TUV_uel, TUP = TUP_uel, TUQ = TUQ_uel, KUI = KUI_uel, KUL = KUL_uel, VUIMAX = VUIMAX_uel, VUIMIN = VUIMIN_uel, KUF = KUF_uel, KFB = KFB_uel, TUL = TUL_uel, TU1 = TU1_uel, TU2 = TU2_uel, TU3 = TU3_uel, TU4 = TU4_uel, P0 = P0_uel, Q0 = Q0_uel, P1 = P1_uel, Q1 = Q1_uel, P2 = P2_uel, Q2 = Q2_uel, P3 = P3_uel, Q3 = Q3_uel, P4 = P4_uel, Q4 = Q4_uel, P5 = P5_uel, Q5 = Q5_uel, P6 = P6_uel, Q6 = Q6_uel, P7 = P7_uel, Q7 = Q7_uel, P8 = P8_uel, Q8 = Q8_uel, P9 = P9_uel, Q9 = Q9_uel, P10 = P10_uel, Q10 = Q10_uel, VULMAX = VULMAX_uel, VULMIN = VULMIN_uel, M0 = M0_uel, M1 = M1_uel, M2 = M2_uel) annotation(Placement(visible = true, transformation(origin={-36,10}, extent = {{-10, -10}, {25, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {40, 35}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-32,-8}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
equation 
  // connect(const.y, uel21.VFB) annotation(Line(points = {{-54.5, -20}, {-49.4305, -20}, {-49.4305, 2.96128}, {-83.5991, 2.96128}, {-83.5991, 11.1617}, {-83.5991, 11.1617}}, color = {0, 0, 127}));
  connect(gensae1.PMECH0, gensae1.PMECH) annotation (Line(points = {{30,-8},{
          28.0182,-8},{28.0182,-6},{30,-6}}, color = {0,0,127}));
  connect(TRIP, gensae1.TRIP) annotation(Line(points = {{40,35},{39.9586,35},{
          39.9586,10},{40,10}}));
  //
  connect(ieeex11.EFD0, gensae1.EFD0)
    annotation (Line(points = {{24,0},{29.8,0}}, color={0,0,127}));
  connect(ieeex11.EFD, gensae1.EFD) annotation (Line(points = {{24,2},{28,2},{
          28,2.8},{30,2.8}}, color = {0,0,127}));
  connect(ieeex11.ETERM0, gensae1.ETERM0) annotation (Line(points = {{24,4},{28,
          4},{28,5},{29.8,5}}, color = {0,0,127}));
  connect(ieeex11.EX_AUX, gensae1.EX_AUX) annotation (Line(points = {{24,6.6},{
          26,6.6},{26,8},{29.8,8}}, color = {0,0,127}));
  connect(uel21.VUEL, ieeex11.VUEL) annotation (Line(points={{-10.3,10},{4,
          10}}, color={0,0,127}));
  connect(const.y, ieeex11.VOTHSG) annotation (Line(points = {{-3.75,5.5},{-2,
          5.5},{-2,4.4},{4,4.4}}, color = {0,0,127}));
  connect(ieeex11.VOEL, ieeex11.VOTHSG) annotation (Line(points = {{4,7.2},{-2,
          7.2},{-2,4.4},{4,4.4}}, color = {0,0,127}));
  connect(ieeex11.XADIFD, gensae1.XADIFD) annotation (Line(points = {{4,13},{-2,
          13},{-2,24},{54,24},{54,5},{50,5}}, color = {0,0,127}));
  connect(uel21.VF, ieeex11.VF) annotation (Line(points={{-45.65,10},{-52,10},{-52,
          30},{28,30},{28,16},{24.4,16}}, color = {0,0,127}));
  connect(uel21.EX_AUX, gensae1.EX_AUX) annotation (Line(points={{-45.65,18},{-54,
          18},{-54,32},{26,32},{26,8},{29.8,8}}, color = {0,0,127}));
  connect(dVREF, ieeex11.dVREF)
    annotation (Line(points={{-32,-8},{0,-8},{0,2},{4,2}}, color={0,0,0}));
  connect(gensae1.p, bus0)
    annotation (Line(points={{40,-10},{40,-18},{56,-18}}, color={0,0,0}));
  connect(gensae1.VI, ieeex11.VI)
    annotation (Line(points={{50,0},{56,0},{56,14},{24,14}}, color={0,0,127}));
  connect(gensae1.VI, uel21.VI) annotation (Line(points={{50,0},{56,0},{56,26},{
          -50,26},{-50,14},{-45.65,14}}, color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {0.341686, -0.113895}, extent = {{-99.4305, 99.4305}, {99.4305, -99.4305}}), Text(origin = {-58.6545, 18.785}, extent = {{143.62, -49.31}, {-24.72, 14.46}}, textString = "GENSAE_IEEEX1_UEL2"), Text(origin = {67.6574, -76.3052}, extent = {{-17.77, 11.62}, {17.77, -11.62}}, textString = "PIN"), Text(origin = {-69.97, 61.46}, extent = {{-17.77, 11.62}, {17.77, -11.62}}, textString = "TRIP"), Text(origin = {-67.0126, -76.3972}, extent = {{-17.77, 11.62}, {31.893, -15.948}}, textString = "dVREF")}));
end GENSAE_IEEEX1_UEL2;
