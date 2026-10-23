%% ========================================================================
%  Singularity analysis for the MODIFIED aerial-manipulator arm
%  ------------------------------------------------------------------------
%  Operational singularity:  det J_3y0(q) = 0,  certified via sigma_min(J_3y0).
%
%  Structural facts (all exact in the clean / on-link-CoM model):
%   * sigma_min(J_3y) = sigma_min(J_3y0)          [ diag(R0,1) orthogonal ]
%   * J_3y0 is INDEPENDENT of q1 (exact) and q4 (exact, on-link CoM)
%       -> the singular set lives entirely in the (q2,q3) plane.
%   * det J_3y0 = +/- det(B_ypsi) * det(B_xz), with
%       det B_ypsi = rho_1x = (1/m)( l3 mu3 cos q23 - l2 mu2 sin q2 - l4 mu4 sin q23 )   [BASE/YAW]
%       det B_xz   = (l2 mu2 / m^2)( l3 mu3 cos q3 - l4 mu4 sin q3 )                      [ELBOW]
%     mu_k = total mass proximal to link-k CoM;  q23 = q2+q3.
%
%  Frames: paper convention, arm HANGS DOWNWARD from base {0}.
%    axes (parent):  h = (e3,e2,e2,e3);   links: l = (-l1 e3,-l2 e3, l3 e1,-l4 e3)
% ========================================================================
clear; clc; close all;
set(groot,'defaultAxesFontName','Times New Roman','defaultTextFontName','Times New Roman');

%% ------------------- 1. PARAMETERS ---------------
e1 = [1;0;0];  e2 = [0;1;0];  e3 = [0;0;1];
d2r = pi/180;                                         % deg->rad (used throughout)

l1 = 0.077;  l2 = 0.128;  l3 = 0.124;  l4 = 0.126;   % link lengths (m)
delta2 = 0.024;                                      % link-2 in-plane bend, forward (+e1) offset (m)
hpar = [e3, e2, e2, e3];                             % axes h1..h4 (parent frame)
Lvec = [-l1*e3, delta2*e1 - l2*e3, l3*e1, -l4*e3];   % link offset vectors l_i in {i}  (l2 = d2 e1 - l2 e3)

m0 = 2.4761;                                         % DRONE mass (Holybro X650) -- shifts singular set
mi = [0.10483, 0.14235, 0.13467, 0.23551];           % OM-X links 1..4 (kg)
mp = 0.2;                                      % grasped payload at link 4 (kg) -- set as needed; moves both branches
mi(4) = mi(4) + mp;                            % payload rides on the terminal link
mass = [m0, mi];  mtot = sum(mass);

% Link CoM offsets c_i in {i}, from O_{i-1} (m) -- your verified values
%        link1     link2     link3     link4(mod)
Ccom = [ 0.0      0.00916   0.09329   -0.00613 ;   % x
        -0.00057 -0.00042  -0.00044    0.0     ;   % y
        -0.02657 -0.10600   0.0       -0.06047 ];  % z

% lim = [ -35 35 ; -90 50 ; -90 50 ; -180 180 ];     % INITIAL GUESS (deg): full joint travel -- run 2a on this to get the q2,q3 box
lim = [ -35 35 ; -40 50 ; -90 50 ; -180 180 ];       % CALIBRATED (deg): q2,q3 box from 2a (EE stays below shoulder plane); comment the line above when using this

% ---- simplified-model parameters (collinear-CoM idealization: c_k = gamma_k l_k) ----
%  Basis for the closed-form singular branches (elbow q3*, yaw det B_ypsi, det B_xz),
%  which BOTH cases plot: exact for the simplified case, reference overlay for the actual.
gam = zeros(1,4); Ccom_simp = zeros(3,4);
for i = 1:4
    gam(i) = (Ccom(:,i).'*Lvec(:,i)) / (Lvec(:,i).'*Lvec(:,i));   % along-link fraction
    Ccom_simp(:,i) = gam(i)*Lvec(:,i);                                % on-link CoM
end
% effective mass proximal to each link-k CoM  (mu1 never enters the determinants)
mu2 = m0+mi(1)+(1-gam(2))*mi(2);
mu3 = m0+mi(1)+mi(2)+(1-gam(3))*mi(3);
mu4 = m0+mi(1)+mi(2)+mi(3)+(1-gam(4))*mi(4);


% ---- analysis knobs: characteristic length, safe-region margins, grid resolution ----
%  The three fields (sigma_min, sigma_max, kappa) are computed from J3y0 with its
%  translational rows scaled by Lchar, so all singular values are DIMENSIONLESS and
%  comparable.  The SAFE region is the sigma_min super-level set (>= eps_sig); sigma_max
%  and kappa = sigma_max/sigma_min are plotted for insight only.
Lchar        = (l1+l2+l3+l4)/2;   % characteristic length (half total reach) -- non-dimensionalizes J3y0
eps_sig_simp = 0.10;              % simplified-case sigma_min keep-out margin (dimensionless)
eps_sig_act  = 0.10;              % actual-case sigma_min keep-out margin (dimensionless)
Nq = 200;                         % (q2,q3) grid resolution -> Nq x Nq maps

% ---- run flags: pick which parts execute (set true/false, then just run the script) ----
run_preliminary = false;    % Section 2: one-time (q2,q3) calibration + sanity checks/figures
run_simplified  = true;    % simplified-CoM (simplified) case: Fig 1-3
run_actual      = true;    % bent-l2 + full-CoM (actual) case: Fig 4-5   (Fig 6 needs BOTH cases)

% ---- shared init for the two cases: (q2,q3) grid, analytic yaw branch, colormap ----
%  Built from the Section-1 `lim` above.  (If you use 2a to re-calibrate `lim`, hard-code
%  the result into `lim` above and keep run_preliminary=false when plotting, so this grid matches.)
if run_simplified || run_actual
    q2v = linspace(lim(2,1),lim(2,2),Nq)*d2r;
    q3v = linspace(lim(3,1),lim(3,2),Nq)*d2r;
    [Q2,Q3] = meshgrid(q2v,q3v);
    RHO = (l3*mu3*cos(Q2+Q3) - l2*mu2*sin(Q2) - l4*mu4*sin(Q2+Q3))/mtot;   % analytic det B_ypsi (yaw branch)
    % "nebula" colormap: dark cosmos (low = danger) -> bright starlight (high = safe)
    anchors = [ 0.015 0.013 0.106 ;   % deep space
              0.180 0.055 0.325 ;   % indigo
              0.435 0.098 0.514 ;   % violet
              0.706 0.157 0.475 ;   % magenta
              0.918 0.396 0.310 ;   % coral
              0.988 0.706 0.447 ;   % amber
              0.996 0.925 0.788 ];  % pale starlight
    xi = linspace(0,1,size(anchors,1)); xf = linspace(0,1,256);
    cmap = [interp1(xi,anchors(:,1),xf,'pchip').' ...
          interp1(xi,anchors(:,2),xf,'pchip').' ...
          interp1(xi,anchors(:,3),xf,'pchip').'];
    cmap = min(max(cmap,0),1);
end


%% ================= 2. PRELIMINARY  (run once: one-time calibration + sanity checks) =================
%  Everything here runs ONCE.  The two case branches below do NOT depend on any
%  printout here; the ONLY value that feeds downstream is the (q2,q3) box in
%  `lim` (set in 2a), which can be HARD-CODED into `lim` in Section 1 once read
%  off -- after which this whole section may be skipped.
if run_preliminary

% ---- 2a. one-time calibration: restrict (q2,q3) so the END-EFFECTOR stays BELOW the shoulder plane (r_e,z <= 0) ----
%  r_e,z is q1- and q4-invariant, so this is purely a (q2,q3) constraint.
%  RUN ONCE, then paste the printed box into `lim` (Section 1) to skip Section 2.
rez = @(q2,q3) -l1 - l2.*cos(q2) - l3.*sin(q2+q3) - l4.*cos(q2+q3);      % EE height
gg = 1001; q2s = linspace(lim(2,1),lim(2,2),gg)*d2r; q3s = linspace(lim(3,1),lim(3,2),gg)*d2r;
[G2,G3] = meshgrid(q2s,q3s); okz = rez(G2,G3) <= 0;                        % admissible (rows q3, cols q2)
colok = all(okz,1); rowok = all(okz,2).';                                    % all-other-axis admissible
q2A = [min(q2s(colok)) max(q2s(colok))]/d2r; q3A = lim(3,:);                 % A: keep full q3, tighten q2
q2B = lim(2,:);  q3B = [min(q3s(rowok)) max(q3s(rowok))]/d2r;                % B: keep full q2, tighten q3
if diff(q2A)*diff(q3A) >= diff(q2B)*diff(q3B)                            % pick larger-area box
    lim(2,:) = q2A; lim(3,:) = q3A;
else
    lim(2,:) = q2B; lim(3,:) = q3B;
end
lim(2,:) = [ceil(lim(2,1)) floor(lim(2,2))];   % round INWARD to integer deg (stays conservative/safe)
lim(3,:) = [ceil(lim(3,1)) floor(lim(3,2))];   % all subsequent grids use this rounded, safe range
fprintf('EE-below-plane box (rounded, r_ez<=0):  q2 in [%.0f, %.0f] deg,  q3 in [%.0f, %.0f] deg\n',lim(2,:),lim(3,:));

% ---- 2b. sanity: collinear-CoM deviation -- measured CoM vs its on-link projection ----
%  (collinearity, c_k = gam_k l_k) is the only thing that separates the two cases:
%  the actual case uses the full measured CoM (Ccom), the simplified case its on-link
%  projection (Ccom_simp = gam_k l_k).  perp = |c - (c.l^)l^| = |Ccom - Ccom_simp| is the
%  T3 error; small perp => the two cases nearly coincide.
fprintf('\n---- CoM-vs-link deviation (collinear-CoM) ----\n');
fprintf('%-5s %-9s %-8s %-13s %-12s\n','link','|c|(mm)','gamma','ang(c,l)deg','perp(mm)');
for i = 1:4
    c  = Ccom(:,i);
    ch = c/norm(c);                             % unit CoM direction
    lh = Lvec(:,i)/norm(Lvec(:,i));             % unit link axis
    angcl = acosd(max(-1,min(1, ch.'*lh)));     % angle between CoM and link axis
    perp  = norm(c - (c.'*lh)*lh);              % perpendicular offset |Ccom - Ccom_simp| (T3 error)
    fprintf('%-5d %-9.2f %-8.3f %-13.2f %-12.3f\n', i, 1e3*norm(c), gam(i), angcl, 1e3*perp);
end

% ---- 2c. sanity: visualize hanging arm at q = 0 (geometry check -- uses ACTUAL CoMs) ----
base = [0;0;0];                                      % plot offset only (arm hangs in -z)
[Rall,~,O,rC,r0e,~,r0c] = fk(zeros(4,1),hpar,Lvec,Ccom,mass,mtot);
h_pose = figure('Color','w','Name','Initial pose q=0 (actual CoM)','Position',[60 80 660 640]);
hold on; grid on; axis equal; s=0.05;
[Plink,Pjnt,Pe] = armpoly(zeros(4,1),hpar,Lvec,Ccom,mass,mtot);   % link 2 drawn bent (two segments)
PL = base + Plink;  PJ = base + Pjnt;  PE = base + Pe;
plot3(PL(1,:),PL(2,:),PL(3,:),'-','Color',[.4 .4 .4],'LineWidth',3);                                  % links (link 2 bent)
plot3(PJ(1,:),PJ(2,:),PJ(3,:),'o','MarkerSize',9,'MarkerFaceColor',[1 .5 0],'MarkerEdgeColor','k');   % joints (none at bend)
plot3(PE(1),PE(2),PE(3),'p','MarkerSize',16,'MarkerFaceColor','y','MarkerEdgeColor','k');             % EE
Pc = base + rC(:,2:5);
plot3(Pc(1,:),Pc(2,:),Pc(3,:),'o','MarkerSize',7,'MarkerFaceColor','m','MarkerEdgeColor','k');
plot3(base(1)+r0c(1),base(2)+r0c(2),base(3)+r0c(3),'^','MarkerSize',12,'MarkerFaceColor',[0 .6 0],'MarkerEdgeColor','k');
lbl={'\{I/0\}','\{1\}','\{2\}','\{3\}','\{4/e\}'};   % {I/0}: inertial {I} coincides with base {0} at base=[0;0;0]
for i=1:5, p=base+[O,r0e]; p=p(:,i); drawTriad(p,Rall{i},s); text(p(1)+0.02,p(2),p(3)+0.02,lbl{i},'FontSize',11,'FontWeight','bold'); end
xlabel('x_I (m)'); ylabel('y_I (m)'); zlabel('z_I (m)'); title('Initial configuration (modified, arm hanging)');
view(35,20); camlight;
hh(1)=plot3(nan,nan,nan,'-','Color',[.4 .4 .4],'LineWidth',3);
hh(2)=plot3(nan,nan,nan,'o','MarkerFaceColor',[1 .5 0],'MarkerEdgeColor','k');
hh(3)=plot3(nan,nan,nan,'p','MarkerFaceColor','y','MarkerEdgeColor','k','MarkerSize',12);
hh(4)=plot3(nan,nan,nan,'o','MarkerFaceColor','m','MarkerEdgeColor','k');
hh(5)=plot3(nan,nan,nan,'^','MarkerFaceColor',[0 .6 0],'MarkerEdgeColor','k');
legend(hh,{'links','joints O_i','end-effector','link CoM','system CoM r_c'},'Location','northeastoutside');
hold off; drawnow;

end   % run_preliminary

%% ================== 3. SIMPLIFIED CASE (collinear CoM) ===================
%% ------------------- FIG 1: SIMPLIFIED (q2,q3) FIELDS -- sigma_min | sigma_max | kappa --
%  Complete singular set (q1=q4=0 is EXACTLY representative). On-link (simplified) CoM.
if run_simplified

q3star = atan2(l3*mu3, l4*mu4);               % elbow branch: det B_xz = 0 (q2-independent)

Smin = zeros(Nq,Nq); Smax = zeros(Nq,Nq); Kap = zeros(Nq,Nq);   % fields (j=q3, i=q2)
for i = 1:Nq
    for j = 1:Nq
        [Smin(j,i),Smax(j,i),Kap(j,i)] = svals(J3y0([0;q2v(i);q3v(j);0],hpar,Lvec,Ccom_simp,mass,mtot),Lchar);
    end
end
[sminv,idx] = min(Smin(:)); [jr,ic] = ind2sub(size(Smin),idx);
q2min = q2v(ic)/d2r; q3min = q3v(jr)/d2r;

h_f1 = figure('Color','w','Name','Fig 1: SIMPLIFIED  sigma_min | sigma_max | kappa','Position',[100 240 1360 470]);
plotfields(Smin,Smax,Kap, q2v,q3v,d2r,lim,cmap, eps_sig_simp, RHO,q3star, q2min,q3min, 'SIMPLIFIED','-');

safe = Smin >= eps_sig_simp;                                     % safe region: sigma_min >= margin
fprintf('\nSIMPLIFIED (q2,q3):\n');
fprintf('  global min sigma_min = %.4e  at  q2=%.1f, q3=%.1f deg\n', sminv, q2min, q3min);
if any(safe(:))
    fprintf('  safe region (sigma_min >= %.3f):\n', eps_sig_simp);
    fprintf('    sigma_min in [%.4f, %.4f]\n', min(Smin(safe)), max(Smin(safe)));
    fprintf('    sigma_max in [%.4f, %.4f]\n', min(Smax(safe)), max(Smax(safe)));
    fprintf('    kappa     in [%.1f, %.1f]\n', min(Kap(safe)),  max(Kap(safe)));
else
    fprintf('  safe region (sigma_min >= %.3f) is EMPTY\n', eps_sig_simp);
end

%% ------------------- FIG 2: singularity branches (elbow + 3-D yaw) ------
h_f2 = figure('Color','w','Name','Fig 2: singularity branches','Position',[560 80 900 380]);

subplot(1,2,1);                                       % elbow: det B_xz vs q3
q3f = linspace(lim(3,1),lim(3,2),400)*d2r;
dBxz = l2*mu2/mtot^2*(l3*mu3*cos(q3f)-l4*mu4*sin(q3f));
plot(q3f/d2r,dBxz,'LineWidth',2); hold on; yline(0,'k:');
plot(q3star/d2r,0,'rp','MarkerFaceColor','r','MarkerSize',10);
text(q3star/d2r-1, 5e-3, sprintf('$q_3^{*}=%.1f^{\\circ}$',q3star/d2r),'Interpreter','latex','FontSize',11,'Color','r','HorizontalAlignment','right');   % singular point
xlabel('$q_3\ (\mathrm{deg})$','Interpreter','latex');
ylabel('$\det \mathbf{B}_{xz}$','Interpreter','latex'); grid on;
title(sprintf('Elbow: $\\det \\mathbf{B}_{xz}=0$'),'Interpreter','latex');
set(gca,'FontName','Times New Roman','FontSize',12);

subplot(1,2,2);                                       % base/yaw: 3-D surface + z=0 cut
surf(Q2/d2r,Q3/d2r,RHO,'EdgeColor','none','FaceAlpha',0.72); hold on;   % translucent so the orange cut reads through
zl = [min(RHO(:)) max(RHO(:))];
patch([lim(2,1) lim(2,2) lim(2,2) lim(2,1)],[lim(3,1) lim(3,1) lim(3,2) lim(3,2)],[0 0 0 0],...
      [0.6 0.6 0.6],'FaceAlpha',0.3,'EdgeColor','none');            % translucent z=0 plane
[~,hc] = contour3(Q2/d2r,Q3/d2r,RHO,[0 0]); set(hc,'LineColor',[0.93 0.46 0.00],'LineWidth',2.5); % zero cut on surface
C = contourc(q2v/d2r,q3v/d2r,RHO,[0 0]);                % floor projection of zero curve
ii = 1; lblxy = [];
while ii<size(C,2)
    n = C(2,ii); xy = C(:,ii+1:ii+n);
    plot3(xy(1,:),xy(2,:),zl(1)*ones(1,n),'k--','LineWidth',1.5);
    if isempty(lblxy), lblxy = xy(:,max(1,round(0.55*n))); end   % anchor for the singular-locus label
    ii = ii+n+1;
end
if ~isempty(lblxy)   % label the singular line with its (q2,q3) equation (det B_ypsi = 0)
    text(lblxy(1),lblxy(2),0.06,'$l_3\mu_3\cos(q_2{+}q_3)-l_2\mu_2\sin q_2-l_4\mu_4\sin(q_2{+}q_3)=0$',...
         'Interpreter','latex','FontSize',10,'Color',[0.93 0.46 0.00],'HorizontalAlignment','center');
end
xlabel('$q_2\ (\mathrm{deg})$','Interpreter','latex');
ylabel('$q_3\ (\mathrm{deg})$','Interpreter','latex');
zlabel('$\det \mathbf{B}_{y\psi}$','Interpreter','latex');
title('Yaw: $\det \mathbf{B}_{y\psi}=0$','Interpreter','latex');
set(gca,'FontName','Times New Roman','FontSize',12);
colormap(gca,cmap); view(-40,25); grid on; axis tight;   % nebula (reuse Fig 1 map)

%% ------------------- FIG 3: TASK-SPACE WORKSPACE --------------------
%  sigma_min is q1- & q4-invariant (clean).  EE position: q4-invariant, but
%  q1 REVOLVES it about z -> the planar (x,z) cross-section (q1=q4=0) sweeps
%  into a solid of revolution; color is constant along each q1-arc.
%  Colorbar upper limit = max sigma_min actually present in each figure (best contrast).

h_f3 = figure('Color','w','Name','Fig 3: task-space workspace','Position',[80 40 1040 500]);
% --- q=0 arm skeleton (actual CoM), planar in x-z; drawn into both panels ---
[Parm,Pjnt,Pe] = armpoly(zeros(4,1),hpar,Lvec,Ccom,mass,mtot);   % Parm: link polyline (link 2 bent); Pjnt: joints; Pe: EE


% ---- LEFT: planar cross-section (q1=q4=0), the generating slice ----
RE = zeros(2,Nq*Nq); SV = zeros(1,Nq*Nq); k = 0;
for i = 1:Nq
    for j = 1:Nq
        [~,~,~,~,re,~,~] = fk([0;q2v(i);q3v(j);0],hpar,Lvec,Ccom_simp,mass,mtot);
        k = k+1; RE(:,k) = [re(1);re(3)]; SV(k) = Smin(j,i);
    end
end
subplot(1,2,1);
cmax3 = max(SV);                                    % largest sigma_min shown in Fig 3 (both panels share it)
scatter(RE(1,:),RE(2,:),10,SV,'filled'); axis equal; hold on;
plot(Parm(1,:),Parm(3,:),'-','Color',[.3 .3 .3],'LineWidth',3);            % links (x-z)
plot(Pjnt(1,:),Pjnt(3,:),'o','MarkerFaceColor',[1 .5 0],'MarkerEdgeColor','k','MarkerSize',7); % joints
plot(Pe(1),Pe(3),'p','MarkerFaceColor','y','MarkerEdgeColor','k','MarkerSize',13);             % EE
colormap(gca,cmap); caxis([0 cmax3]);
cb = colorbar; cb.Label.Interpreter = 'latex'; cb.Label.String = '$\sigma_{\mathrm{min}}$';
xlabel('$r_{e,x}\ (\mathrm{m})$','Interpreter','latex');
ylabel('$r_{e,z}\ (\mathrm{m})$','Interpreter','latex');
title('Cross-section ($q_1=q_4=0$)','Interpreter','latex');
set(gca,'FontName','Times New Roman','FontSize',12);

% ---- RIGHT: 3-D revolution over q1 (q4 verified position-invariant) ----
step = 5; ia = 1:step:Nq; ja = 1:step:Nq;               % subsample the (q2,q3) grid
M = numel(ia)*numel(ja); RE0 = zeros(3,M); SV0 = zeros(1,M); c = 0;
for a = 1:numel(ia)
    for b = 1:numel(ja)
        [~,~,~,~,re,~,~] = fk([0;q2v(ia(a));q3v(ja(b));0],hpar,Lvec,Ccom_simp,mass,mtot);
        c = c+1; RE0(:,c) = re; SV0(c) = Smin(ja(b),ia(a));
    end
end

q1v = linspace(lim(1,1),lim(1,2),15)*d2r;           % base-yaw range
X = [];Y = [];Z = [];CC = [];
for q1 = q1v
    Rz = [cos(q1) -sin(q1) 0; sin(q1) cos(q1) 0; 0 0 1];
    P = Rz*RE0; X = [X P(1,:)]; Y = [Y P(2,:)]; Z = [Z P(3,:)]; CC = [CC SV0]; %#ok<AGROW>
end
subplot(1,2,2);
keep = Y>=-1e-9;                                  % cutaway: keep far half (arm on the y=0 cut face)
scatter3(X(keep),Y(keep),Z(keep),8,CC(keep),'filled',...
         'MarkerFaceAlpha',0.5,'MarkerEdgeAlpha',0); hold on;
plot3(Parm(1,:),Parm(2,:),Parm(3,:),'-','Color',[0 0 0],'LineWidth',4);                 % arm links (opaque, on cut face)
plot3(Pjnt(1,:),Pjnt(2,:),Pjnt(3,:),'o','MarkerFaceColor',[1 .5 0],'MarkerEdgeColor','k','MarkerSize',8);
plot3(Pe(1),Pe(2),Pe(3),'p','MarkerFaceColor','y','MarkerEdgeColor','k','MarkerSize',15);
colormap(gca,cmap); caxis([0 cmax3]);
cb = colorbar; cb.Label.Interpreter = 'latex'; cb.Label.String = '$\sigma_{\mathrm{min}}$';
xlabel('$r_{e,x}\ (\mathrm{m})$','Interpreter','latex');
ylabel('$r_{e,y}\ (\mathrm{m})$','Interpreter','latex');
zlabel('$r_{e,z}\ (\mathrm{m})$','Interpreter','latex');
title('Revolution over $q_1$','Interpreter','latex');
set(gca,'FontName','Times New Roman','FontSize',12);
axis equal;                                  % equal data aspect
xlim([min(X) max(X)]); ylim([min(Y) max(Y)]); zlim([min(Z) max(Z)]);  % full ranges; empty near half kept
view(-25,15); grid on; camlight; lighting gouraud;

end   % run_simplified

%% ================== 4. ACTUAL CASE (full measured CoM) ===================
%  Same fk/J3y0/Lvec as the simplified case; only the CoM differs -- full measured Ccom vs on-link Ccom_simp.
if run_actual
q3star = atan2(l3*mu3, l4*mu4);               % elbow branch (simplified analytic), drawn as reference overlay

SminA = zeros(Nq,Nq); SmaxA = zeros(Nq,Nq); KapA = zeros(Nq,Nq);
for i = 1:Nq
    for j = 1:Nq
        [SminA(j,i),SmaxA(j,i),KapA(j,i)] = svals(J3y0([0;q2v(i);q3v(j);0],hpar,Lvec,Ccom,mass,mtot),Lchar);
    end
end
[sminvA,idxA] = min(SminA(:)); [jrA,icA] = ind2sub(size(SminA),idxA);
q2minA = q2v(icA)/d2r; q3minA = q3v(jrA)/d2r;

% ---- q4 worst-case: the true (off-axis) c_4 reintroduces q4 into J3y0 ----
%  The per-point worst q4 is BIMODAL over the box (~-180 deg and ~0 deg), but a single
%  constant q4 = q4_worst reproduces the worst-case sigma_min field to <~1.1% of the margin
%  with NO change in safe/unsafe classification.  So all worst-case fields are evaluated at
%  this one q4 (single pass, same cost as the q4=0 fields).  q4_worst must be set BEFORE the
%  fields below.  Set run_q4search=true to run the one-time per-point search that justifies it.
q4_worst     = -180;     % [deg] worst-case q4 -- set manually from the search (run_q4search)
run_q4search = false;    % true -> one-time per-point q4 search (prints the gap vs q4_worst)

q4w = q4_worst*d2r;
SminAw = zeros(Nq,Nq); SmaxAw = zeros(Nq,Nq); KapAw = zeros(Nq,Nq);
for i = 1:Nq
    for j = 1:Nq
        [SminAw(j,i),SmaxAw(j,i),KapAw(j,i)] = svals(J3y0([0;q2v(i);q3v(j);q4w],hpar,Lvec,Ccom,mass,mtot),Lchar);
    end
end
[sminvAw,idxAw] = min(SminAw(:)); [jrAw,icAw] = ind2sub(size(SminAw),idxAw);
q2minAw = q2v(icAw)/d2r; q3minAw = q3v(jrAw)/d2r;

% sigma_min change from q4=0 to q4_worst: largest drop, where, and as % of value & margin
dS = SminA - SminAw; [dmax,di] = max(dS(:)); [jd,idd] = ind2sub(size(dS),di);
fprintf('\nACTUAL q4 effect (q4=0 -> q4_worst=%.0f deg):\n', q4_worst);
fprintf('  sigma_min %.4f -> %.4f  at (q2,q3)=(%.1f, %.1f) deg\n', ...
        SminA(jd,idd), SminAw(jd,idd), q2v(idd)/d2r, q3v(jd)/d2r);
fprintf('  change = %.2f%% of its q4=0 value, %.2f%% of the margin (eps=%.3f)\n', ...
        100*dmax/SminA(jd,idd), 100*dmax/eps_sig_act, eps_sig_act);

if run_q4search
    Nq4 = 50; q4grid = linspace(-pi,pi,Nq4);                     % fine-enough q4 grid (smooth in q4)
    SminTrue = zeros(Nq,Nq);
    for i = 1:Nq
        for j = 1:Nq
            SminTrue(j,i) = svals_q4wc(q2v(i),q3v(j), q4grid, hpar,Lvec,Ccom,mass,mtot, Lchar);
        end
    end
    gap = SminAw - SminTrue;   % >=0: how much constant q4_worst overestimates the true per-point worst
    fprintf('  [search] q4_worst vs true per-point worst: max gap %.2f%% of margin, %d classification flips\n', ...
            100*max(gap(:))/eps_sig_act, nnz((SminAw>=eps_sig_act)~=(SminTrue>=eps_sig_act)));
end

%% ------------------- FIG 4: ACTUAL (q2,q3) FIELDS -- top: q4=0 | bottom: worst-case q4 --
h_f4 = figure('Color','w','Name','Fig 4: ACTUAL  sigma_min | sigma_max | kappa (q4=0 vs worst q4)','Position',[100 60 1360 900]);
plotfields(SminA ,SmaxA ,KapA ,  q2v,q3v,d2r,lim,cmap, eps_sig_act, RHO,q3star, q2minA ,q3minA , '$q_4{=}0$'  ,'-', 2,1);
plotfields(SminAw,SmaxAw,KapAw, q2v,q3v,d2r,lim,cmap, eps_sig_act, RHO,q3star, q2minAw,q3minAw, 'worst $q_4$','-', 2,2);

safe = SminA >= eps_sig_act;                                     % safe region: sigma_min >= margin
fprintf('\nACTUAL (q2,q3):\n');
fprintf('  global min sigma_min = %.4e  at  q2=%.1f, q3=%.1f deg\n', sminvA, q2minA, q3minA);
if any(safe(:))
    fprintf('  safe region (sigma_min >= %.3f):\n', eps_sig_act);
    fprintf('    sigma_min in [%.4f, %.4f]\n', min(SminA(safe)), max(SminA(safe)));
    fprintf('    sigma_max in [%.4f, %.4f]\n', min(SmaxA(safe)), max(SmaxA(safe)));
    fprintf('    kappa     in [%.1f, %.1f]\n', min(KapA(safe)),  max(KapA(safe)));
else
    fprintf('  safe region (sigma_min >= %.3f) is EMPTY\n', eps_sig_act);
end

%% ------------------- FIG 5: ACTUAL task-space workspace (worst-case q4) -------------
%  EE position r_0e is q4-INVARIANT (joint 4 spins about its own link axis), so worst-case
%  q4 moves neither the workspace shape nor the skeleton -- it only lowers the color
%  (sigma_min = SminAw).  The joint-4 twist is shown on the EE FRAME (right panel): tool
%  axis b3 (blue) is fixed, b1 sweeps a ring as q4 spins, worst q4 marked bold red.
h_f5 = figure('Color','w','Name','Fig 5: ACTUAL task-space workspace (worst q4)','Position',[80 40 1040 500]);
[Parma,Pjnta,Pea] = armpoly(zeros(4,1),hpar,Lvec,Ccom,mass,mtot);   % skeleton (q4-invariant)
q4home = q4_worst*d2r;                                                       % hardcoded worst-case q4 (no re-search)
R0home = fk(zeros(4,1),hpar,Lvec,Ccom,mass,mtot);                            % rotations at home (R_3^0 = R0home{4})
% LEFT: planar cross-section (color = worst-case sigma_min)
REa = zeros(2,Nq*Nq); SVa = zeros(1,Nq*Nq); k = 0;
for i = 1:Nq
    for j = 1:Nq
        [~,~,~,~,re,~,~] = fk([0;q2v(i);q3v(j);0],hpar,Lvec,Ccom,mass,mtot);   % positions q4-invariant
        k = k+1; REa(:,k) = [re(1);re(3)]; SVa(k) = SminAw(j,i);
    end
end
subplot(1,2,1);
cmax5 = max(SVa);                                   % largest sigma_min shown in Fig 5 (both panels share it)
scatter(REa(1,:),REa(2,:),10,SVa,'filled'); axis equal; hold on;
plot(Parma(1,:),Parma(3,:),'-','Color',[.3 .3 .3],'LineWidth',3);
plot(Pjnta(1,:),Pjnta(3,:),'o','MarkerFaceColor',[1 .5 0],'MarkerEdgeColor','k','MarkerSize',7);
plot(Pea(1),Pea(3),'p','MarkerFaceColor','y','MarkerEdgeColor','k','MarkerSize',13);
colormap(gca,cmap); caxis([0 cmax5]);
cb = colorbar; cb.Label.Interpreter = 'latex'; cb.Label.String = '$\sigma_{\mathrm{min}}$';
xlabel('$r_{e,x}\ (\mathrm{m})$','Interpreter','latex');
ylabel('$r_{e,z}\ (\mathrm{m})$','Interpreter','latex');
title('Cross-section ($q_1=0$, worst $q_4$)','Interpreter','latex');
set(gca,'FontName','Times New Roman','FontSize',12);
% RIGHT: revolution over q1 (color = worst-case sigma_min; EE frame shows the q4 twist)
step = 5; ia = 1:step:Nq; ja = 1:step:Nq;
M = numel(ia)*numel(ja); RE0a = zeros(3,M); SV0a = zeros(1,M); c = 0;
for a = 1:numel(ia)
    for b = 1:numel(ja)
        [~,~,~,~,re,~,~] = fk([0;q2v(ia(a));q3v(ja(b));0],hpar,Lvec,Ccom,mass,mtot);
        c = c+1; RE0a(:,c) = re; SV0a(c) = SminAw(ja(b),ia(a));
    end
end
q1v = linspace(lim(1,1),lim(1,2),15)*d2r;
Xa = [];Ya = [];Za = [];CCa = [];
for q1 = q1v
    Rz = [cos(q1) -sin(q1) 0; sin(q1) cos(q1) 0; 0 0 1];
    P = Rz*RE0a; Xa = [Xa P(1,:)]; Ya = [Ya P(2,:)]; Za = [Za P(3,:)]; CCa = [CCa SV0a]; %#ok<AGROW>
end
subplot(1,2,2);
keepa = Ya>=-1e-9;
scatter3(Xa(keepa),Ya(keepa),Za(keepa),8,CCa(keepa),'filled','MarkerFaceAlpha',0.5,'MarkerEdgeAlpha',0); hold on;
plot3(Parma(1,:),Parma(2,:),Parma(3,:),'-','Color',[0 0 0],'LineWidth',4);
plot3(Pjnta(1,:),Pjnta(2,:),Pjnta(3,:),'o','MarkerFaceColor',[1 .5 0],'MarkerEdgeColor','k','MarkerSize',8);
plot3(Pea(1),Pea(2),Pea(3),'p','MarkerFaceColor','y','MarkerEdgeColor','k','MarkerSize',15);
eeframe(Pea, q4home, R0home{4}, 0.06);              % joint-4 twist ring; worst q4 bold red
text(Pea(1),Pea(2),Pea(3)-0.06,sprintf('worst $q_4=%.0f^{\\circ}$',q4home/d2r),...
     'Interpreter','latex','FontSize',11,'HorizontalAlignment','center');
colormap(gca,cmap); caxis([0 cmax5]);
cb = colorbar; cb.Label.Interpreter = 'latex'; cb.Label.String = '$\sigma_{\mathrm{min}}$';
xlabel('$r_{e,x}\ (\mathrm{m})$','Interpreter','latex');
ylabel('$r_{e,y}\ (\mathrm{m})$','Interpreter','latex');
zlabel('$r_{e,z}\ (\mathrm{m})$','Interpreter','latex');
title('Revolution over $q_1$ (worst $q_4$)','Interpreter','latex');
set(gca,'FontName','Times New Roman','FontSize',12);
axis equal; xlim([min(Xa) max(Xa)]); ylim([min(Ya) max(Ya)]); zlim([min(Za) max(Za)]);
view(-25,15); grid on; camlight; lighting gouraud;

end   % run_actual

%% ================== 5. SIMPLIFIED vs ACTUAL (comparison) ===================
if run_simplified && run_actual
D = SminAw - Smin;                                           % simplified vs actual-worst sigma_min
[dmax,di] = max(abs(D(:))); [jd,idd] = ind2sub(size(D),di);
fprintf('\nSIMPLIFIED vs ACTUAL-WORST sigma_min:  max|d| = %.4e  at q2=%.1f, q3=%.1f deg  (mean %.2e)\n',...
        dmax, q2v(idd)/d2r, q3v(jd)/d2r, mean(abs(D(:))));
fprintf('  = %.0f%% of margin (eps=%.3f)\n', 100*dmax/eps_sig_act, eps_sig_act);

smref = max([Smin(:); SminAw(:)]);                          % shared color scale for the two panels
h_f6 = figure('Color','w','Name','Fig 6: simplified vs actual-worst (sigma_min)','Position',[100 120 1260 380]);
subplot(1,3,1);
imagesc(q2v/d2r,q3v/d2r,Smin); axis xy; colormap(gca,cmap); caxis([0 smref]);
cb = colorbar; cb.Label.Interpreter = 'latex'; cb.Label.String = '$\sigma_{\mathrm{min}}$';
xlabel('$q_2\ (\mathrm{deg})$','Interpreter','latex'); ylabel('$q_3\ (\mathrm{deg})$','Interpreter','latex');
title('Simplified $\sigma_{\mathrm{min}}$','Interpreter','latex'); set(gca,'FontName','Times New Roman','FontSize',12);
subplot(1,3,2);
imagesc(q2v/d2r,q3v/d2r,SminAw); axis xy; colormap(gca,cmap); caxis([0 smref]);
cb = colorbar; cb.Label.Interpreter = 'latex'; cb.Label.String = '$\sigma_{\mathrm{min}}$';
xlabel('$q_2\ (\mathrm{deg})$','Interpreter','latex'); ylabel('$q_3\ (\mathrm{deg})$','Interpreter','latex');
title('Actual worst $\sigma_{\mathrm{min}}$','Interpreter','latex'); set(gca,'FontName','Times New Roman','FontSize',12);
subplot(1,3,3);
imagesc(q2v/d2r,q3v/d2r,abs(D)); axis xy; colormap(gca,cmap); hold on;
plot(q2v(idd)/d2r,q3v(jd)/d2r,'p','MarkerFaceColor',[1 0.9 0],'MarkerEdgeColor','k','MarkerSize',13);
cb = colorbar; cb.Label.Interpreter = 'latex'; cb.Label.String = '$|\Delta\sigma_{\mathrm{min}}|$';
xlabel('$q_2\ (\mathrm{deg})$','Interpreter','latex'); ylabel('$q_3\ (\mathrm{deg})$','Interpreter','latex');
title('$|\Delta\sigma_{\mathrm{min}}|$','Interpreter','latex');
set(gca,'FontName','Times New Roman','FontSize',12);

end   % run_simplified && run_actual

%% ================== SAVE FIGURES -> image/singularity ===================
%  Saves ONLY the figures created by the enabled run_* cases (each captured by
%  its own handle above) into <project root>/image/singularity/.  Path is
%  anchored to THIS script's location, so it works regardless of current folder.
sing_dir = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'image', 'singularity');
if ~exist(sing_dir,'dir'), mkdir(sing_dir); end

% (handle var, output file name) -- only the ones that exist are written
savelist = {};
if exist('h_pose','var'), savelist(end+1,:) = {h_pose, 'initial_pose_q0'};          end %#ok<*SAGROW>
if exist('h_f1'  ,'var'), savelist(end+1,:) = {h_f1,   'fig1_simplified_fields'};    end
if exist('h_f2'  ,'var'), savelist(end+1,:) = {h_f2,   'fig2_singularity_branches'}; end
if exist('h_f3'  ,'var'), savelist(end+1,:) = {h_f3,   'fig3_taskspace_workspace'};  end
if exist('h_f4'  ,'var'), savelist(end+1,:) = {h_f4,   'fig4_actual_fields'};        end
if exist('h_f5'  ,'var'), savelist(end+1,:) = {h_f5,   'fig5_actual_taskspace'};     end
if exist('h_f6'  ,'var'), savelist(end+1,:) = {h_f6,   'fig6_simplified_vs_actual'}; end

nsaved = 0;
for k = 1:size(savelist,1)
    fh = savelist{k,1};
    if ~ishghandle(fh), continue; end               % skip if the user closed it
    outfile = fullfile(sing_dir, [savelist{k,2} '.png']);
    try
        exportgraphics(fh, outfile, 'Resolution', 200);
    catch
        print(fh, outfile, '-dpng', '-r200');        % older MATLAB
    end
    nsaved = nsaved + 1;
end
fprintf('Saved %d figure(s) to %s\n', nsaved, sing_dir);

%% ======================= LOCAL FUNCTIONS ===============================
function [smin,smax,kappa] = svals(J, Lchar)
%  Scale the translational rows (length units) by Lchar so ALL singular values are
%  dimensionless and comparable, then return sigma_min, sigma_max, kappa = smax/smin.
    J(1:3,:) = J(1:3,:)/Lchar;
    s = svd(J);
    smin = s(end); smax = s(1); kappa = smax/smin;
end

function [smin,smax,kappa,q4wc] = svals_q4wc(q2,q3, q4grid, hpar,Lvec,Ccom,mass,mtot, Lchar) %#ok<DEFNU>
%  Worst-case over q4 at fixed (q2,q3), ACTUAL CoM: keep the q4 that MINIMIZES sigma_min
%  (used only by the one-time run_q4search block that justifies the hardcoded q4_worst)
%  (the safe-region metric); report sigma_max, kappa at that same q4.  With the true
%  off-axis c_4, J3y0 depends on q4, so this certifies the (q2,q3) point over all q4.
    smin = inf; smax = 0; kappa = inf; q4wc = 0;
    for q4 = q4grid
        [sn,sx,kp] = svals(J3y0([0;q2;q3;q4],hpar,Lvec,Ccom,mass,mtot),Lchar);
        if sn < smin, smin = sn; smax = sx; kappa = kp; q4wc = q4; end
    end
end

function plotfields(Smin,Smax,Kap, q2v,q3v,d2r,lim,cmap, epsm, RHO,q3star, q2m,q3m, tag,bstyle, nrows,row)
%  3-panel (q2,q3) view of one case: sigma_min (with safe-region margin + analytic branches),
%  sigma_max, and kappa = sigma_max/sigma_min -- all shown over their FULL data range (no cap).
%  nrows,row (optional, default 1,1): place the 3 panels in row `row` of an nrows x 3 grid.
    if nargin < 16, nrows = 1; row = 1; end
    base = (row-1)*3;
    q2d = q2v/d2r; q3d = q3v/d2r; box_ax = [lim(2,:) lim(3,:)];
    % panel 1: sigma_min -- the safe-region metric
    ax1 = subplot(nrows,3,base+1); hold(ax1,'on'); box(ax1,'on');
    imagesc(q2d,q3d,Smin); axis xy; colormap(ax1,cmap); caxis([0 max(Smin(:))]);
    cb = colorbar; cb.Label.Interpreter = 'latex'; cb.Label.String = '$\sigma_{\mathrm{min}}$';
    [~,hE] = contour(q2d,q3d,Smin,[epsm epsm]); set(hE,'LineColor',[0 1 1],'LineStyle',bstyle,'LineWidth',1.8);
    [~,hY] = contour(q2d,q3d,RHO,[0 0]);        set(hY,'LineColor',[0.93 0.46 0.00],'LineStyle',bstyle,'LineWidth',2.0);
    plot(lim(2,:),[q3star q3star]/d2r,bstyle,'Color',[0.10 0.65 0.15],'LineWidth',2.0);
    plot(q2m,q3m,'p','MarkerFaceColor',[1 0.9 0],'MarkerEdgeColor','k','MarkerSize',13);
    xlabel('$q_2\ (\mathrm{deg})$','Interpreter','latex'); ylabel('$q_3\ (\mathrm{deg})$','Interpreter','latex');
    title([tag ': $\sigma_{\mathrm{min}}$'],'Interpreter','latex');
    set(ax1,'FontName','Times New Roman','FontSize',12); axis(ax1,box_ax);
    % panel 2: sigma_max
    ax2 = subplot(nrows,3,base+2); hold(ax2,'on'); box(ax2,'on');
    imagesc(q2d,q3d,Smax); axis xy; colormap(ax2,cmap); caxis([0 max(Smax(:))]);
    cb = colorbar; cb.Label.Interpreter = 'latex'; cb.Label.String = '$\sigma_{\mathrm{max}}$';
    [~,hb] = contour(q2d,q3d,Smin,[epsm epsm]); set(hb,'LineColor',[0 1 1],'LineStyle',bstyle,'LineWidth',1.5);   % safe-region boundary (sigma_min = margin)
    xlabel('$q_2\ (\mathrm{deg})$','Interpreter','latex'); ylabel('$q_3\ (\mathrm{deg})$','Interpreter','latex');
    title([tag ': $\sigma_{\mathrm{max}}$'],'Interpreter','latex');
    set(ax2,'FontName','Times New Roman','FontSize',12); axis(ax2,box_ax);
    % panel 3: kappa = sigma_max / sigma_min
    ax3 = subplot(nrows,3,base+3); hold(ax3,'on'); box(ax3,'on');
    imagesc(q2d,q3d,Kap); axis xy; colormap(ax3,cmap); caxis([1 max(Kap(:))]);   % full range, no cap
    set(ax3,'ColorScale','log');   % log color scale: colorbar reads true kappa (1,10,100,...) on log-spaced ticks
    cb = colorbar; cb.Label.Interpreter = 'latex'; cb.Label.String = '$\kappa$';
    [~,hb] = contour(q2d,q3d,Smin,[epsm epsm]); set(hb,'LineColor',[0 1 1],'LineStyle',bstyle,'LineWidth',1.5);   % safe-region boundary (sigma_min = margin)
    xlabel('$q_2\ (\mathrm{deg})$','Interpreter','latex'); ylabel('$q_3\ (\mathrm{deg})$','Interpreter','latex');
    title([tag ': $\kappa=\sigma_{\mathrm{max}}/\sigma_{\mathrm{min}}$'],'Interpreter','latex');
    set(ax3,'FontName','Times New Roman','FontSize',12); axis(ax3,box_ax);
end

function [Rall,h0,O,rC,r0e,b3e,r0c] = fk(q,hpar,Lvec,Ccom,mass,mtot)
    Rall = {eye(3)}; R = eye(3);
    for k = 1:4, R = R*expSO3(hpar(:,k),q(k)); Rall{end+1} = R; end %#ok<AGROW>
    h0 = zeros(3,4); O = zeros(3,5); rC = zeros(3,5);
    for k = 1:4, h0(:,k) = Rall{k}*hpar(:,k); end
    for i = 1:4, O(:,i+1) = O(:,i)+Rall{i+1}*Lvec(:,i); end
    for i = 1:4, rC(:,i+1) = O(:,i)+Rall{i+1}*Ccom(:,i); end
    r0e = O(:,5); b3e = Rall{5}*[0;0;1];
    r0c = (mass(2)*rC(:,2)+mass(3)*rC(:,3)+mass(4)*rC(:,4)+mass(5)*rC(:,5))/mtot;
end

function J = J3y0(q,hpar,Lvec,Ccom,mass,mtot)
%  J3y0 = [ h_k^0 x rho_k ; (b_3e^0)^T h_k^0 ]_{k=1..n}
%  rho_k = d_e^0 + (1/m) sum_{i=0}^{k-1} m_i (r_0i^0 - r_{0,O_{k-1}}^0),  d_e^0 = r_0e^0 - r_0c^0
    [~,h0,O,rC,r0e,b3e,r0c] = fk(q,hpar,Lvec,Ccom,mass,mtot);
    de = r0e - r0c;                              % d_e^0
    At = zeros(3,4);
    for k = 1:4
        rho = de;                                % O(:,k)=r_{0,O_{k-1}};  rC(:,1)=base CoM=0 (i=0 term)
        for i = 0:k-1
            rho = rho + mass(i+1)*(rC(:,i+1) - O(:,k))/mtot;
        end
        At(:,k) = cross(h0(:,k), rho);           % h_k^0 x rho_k  (top three rows)
    end
    yaw = b3e.'*h0;                              % (b_3e^0)^T h_k^0  (bottom row)
    J = [At; yaw];
end

function [Plink, Pjoint, Pee] = armpoly(q,hpar,Lvec,Ccom,mass,mtot)
%  Arm drawing polyline with the bent link 2 rendered as TWO straight segments.
%  Plink : ordered points to connect -- link 2 = O_1 -> bend corner -> O_2  (NO marker at the corner).
%  Pjoint: joint origins O_0..O_3 (orange markers).   Pee: end-effector (star).
    [Rall,~,O,~,r0e,~,~] = fk(q,hpar,Lvec,Ccom,mass,mtot);
    Ob = O(:,2) + Rall{3}*[0; 0; Lvec(3,2)];             % link-2 corner: drop the vertical part first, then step to O_2
                                                         % (forward-first instead: Rall{3}*[Lvec(1,2);0;0])
    Plink  = [O(:,1), O(:,2), Ob, O(:,3), O(:,4), r0e];  % O_0, O_1, bend, O_2, O_3, EE
    Pjoint = O(:,1:4);                                   % O_0..O_3
    Pee    = r0e;
end

function R = expSO3(h,q)
    h = h/norm(h); K = [0 -h(3) h(2); h(3) 0 -h(1); -h(2) h(1) 0];
    R = eye(3)+sin(q)*K+(1-cos(q))*(K*K);
end

function drawTriad(p,R,s)
    col = {'r','g','b'};
    for a = 1:3, v = R(:,a)*s; quiver3(p(1),p(2),p(3),v(1),v(2),v(3),0,col{a},'LineWidth',2,'MaxHeadSize',0.6); end
end

function eeframe(p, q4w, R3, s)
%  Visualize joint-4 rotation at the end-effector p.  The tool axis b3 = R3*e3 (blue) is
%  q4-INVARIANT; b1 = R3*R_z(q4)*e1 sweeps a ring as q4 spins over [-pi,pi].  Draw the ring
%  (pale) + faint spokes, the fixed tool axis (blue), and the worst-case q4 spoke (bold red).
    th = linspace(-pi,pi,73);
    ring = R3*[s*cos(th); s*sin(th); zeros(1,numel(th))];
    plot3(p(1)+ring(1,:),p(2)+ring(2,:),p(3)+ring(3,:),'-','Color',[.75 .75 .80],'LineWidth',1.0);
    for q4 = linspace(-pi,pi,13)
        b1 = R3*[s*cos(q4); s*sin(q4); 0];
        plot3(p(1)+[0 b1(1)],p(2)+[0 b1(2)],p(3)+[0 b1(3)],'-','Color',[.82 .82 .87],'LineWidth',0.5);
    end
    b3 = R3*[0;0;s];  quiver3(p(1),p(2),p(3),b3(1),b3(2),b3(3),0,'b','LineWidth',2.2,'MaxHeadSize',0.6);
    b1w = R3*[s*cos(q4w); s*sin(q4w); 0];
    quiver3(p(1),p(2),p(3),b1w(1),b1w(2),b1w(3),0,'r','LineWidth',2.6,'MaxHeadSize',0.7);
end