#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    trajRadius_=_trajRadius;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

void KDLPlanner::trapezoidal_vel(double t,double tc,
                        double &s,
                        double &sdot,
                        double &sdotdot
                        )
{
    // Calcolo il valore costante per l'accelerazione massima (s_dotdot_c) 
    // basato sulla durata totale della traiettoria (trajDuration_)
    double sdotdot_c = 4.5 / (std::pow(trajDuration_, 2));
    // Se il tempo t è inferiore al tempo di accelerazione (tc), 
    if (t <= tc) {
        // Calcolo la posizione (s), velocità (s_d) e accelerazione (s_dd) durante la fase di accelerazione
        s = 0.5 * sdotdot_c * std::pow(t, 2);   // Posizione in funzione del tempo
        sdot = sdotdot_c * t;                    // Velocità in funzione del tempo
        sdotdot = sdotdot_c;                       // Accelerazione costante

    // Se t è compreso tra il tempo di accelerazione e decelerazione, si trova nella fase di velocità costante
    } else if (t <= trajDuration_ - tc) {
        // Durante questa fase, la velocità è costante e l'accelerazione è zero
        s = sdotdot_c * tc * (t - tc / 2);    // Posizione in funzione del tempo, considerando la velocità costante
        sdot = sdotdot_c * tc;                 // Velocità costante
        sdotdot = 0;                            // Accelerazione nulla

    // Se t è maggiore del tempo finale meno tc, si trova nella fase di decelerazione
    } else {
        // Calcolo la posizione (s), velocità (s_d) e accelerazione (s_dd) durante la fase di decelerazione
        s = 1 - 0.5 * sdotdot_c * std::pow(trajDuration_ - t, 2); // Posizione alla fine del movimento
        sdot = sdotdot_c * (trajDuration_ - t);  // Velocità decrescente con il tempo
        sdotdot = -sdotdot_c;                      // Accelerazione negativa durante la decelerazione
    }
}

void KDLPlanner::cubic_polinomial(double t, double &s, double &s_d, double &s_dd) {
    // Coefficienti a_2 e a_3 per il polinomio cubico 

    // Questi coefficienti sono derivati dai vincoli di posizione e velocità agli estremi del movimento.
    // a0=0 perche la posizione iniziale è assunta nulla
    // a1=0 perche la velocita iniziale è assunta nulla
    double a_2 = 3 / (std::pow(trajDuration_, 2));   // Coefficiente per il termine t^2 posizione finale =1
    double a_3 = -2 / (std::pow(trajDuration_, 3));  // Coefficiente per il termine t^3 velocita  finale =0

    // Calcolo della curvilinea abscissa s in funzione del tempo t (posizione lungo la traiettoria)
    // La posizione è una combinazione di t^3 e t^2, con coefficienti a_3 e a_2.
    s = a_3 * std::pow(t, 3) + a_2 * std::pow(t, 2);

    // Calcolo della velocità s_d (derivata prima di s rispetto al tempo)
    // La velocità è la derivata prima del polinomio, quindi 3*a_3 * t^2 + 2*a_2 * t
    s_d = 3 * a_3 * std::pow(t, 2) + 2 * a_2 * t;

    // Calcolo dell'accelerazione s_dd (derivata seconda di s rispetto al tempo)
    // L'accelerazione è la derivata seconda del polinomio, quindi 6*a_3 * t + 2*a_2
    s_dd = 6 * a_3 * t + 2 * a_2;
}


KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner :: compute_trajectory_lin_trap ( double time )
{
    double s , s_d , s_dd ;
    trapezoidal_vel ( time ,accDuration_, s , s_d , s_dd );
    trajectory_point traj ;
    traj . pos = trajInit_ + s *( trajEnd_ - trajInit_ ) ;
    traj . vel = s_d *( trajEnd_ - trajInit_ ) ;
    traj . acc = s_dd *( trajEnd_ - trajInit_ ) ;
    return traj ;
}

trajectory_point KDLPlanner :: compute_trajectory_lin_pol ( double time )
{
    double s , s_d , s_dd ;
    cubic_polinomial ( time ,s , s_d , s_dd ) ;
    trajectory_point traj ;
    traj . pos = trajInit_ + s *( trajEnd_ - trajInit_ ) ;
    traj . vel = s_d *( trajEnd_ - trajInit_ ) ;
    traj . acc = s_dd *( trajEnd_ - trajInit_ ) ;
    return traj ;
}

trajectory_point KDLPlanner::compute_trajectory_cir_trap(double time)
{

    double s , s_d , s_dd ;
    trapezoidal_vel ( time ,accDuration_, s , s_d , s_dd ); //1 è un valore a caso
    trajectory_point traj;
    if ( time <= trajDuration_)
    {
        traj.pos.x() = trajInit_.x();
        traj.pos.y() = trajInit_.y() - trajRadius_  * cos(2 * 3.14 * s) + trajRadius_ ;
        traj.pos.z() = trajInit_.z() - trajRadius_  * sin(2 * 3.14 * s);

        traj.vel.y() = trajRadius_  * 2 * 3.14 * s_d * sin(2 * 3.14 * s);
        traj.vel.z() = - trajRadius_  * 2 * 3.14 * s_d * cos(2 * 3.14 * s);

        traj.acc.y() = trajRadius_  * (2 * 3.14) * s_dd * sin(2 * 3.14 * s) + trajRadius_  * (2 * 3.14) * (2 * 3.14) * std::pow(s_d, 2) * cos(2 * 3.14 * s);
        traj.acc.z() = trajRadius_  * (2 * 3.14) * (2 * 3.14) * std::pow(s_d, 2) * sin(2 * 3.14 * s) - trajRadius_  * (2 * 3.14) * s_dd * cos(2 * 3.14 * s);
    }
    return traj;

}

trajectory_point KDLPlanner::compute_trajectory_cir_pol (double time)
{

    double s , s_d , s_dd ;
    cubic_polinomial ( time ,s , s_d , s_dd ) ;
    trajectory_point traj;
    if ( time <= trajDuration_)
    {
        traj.pos.x() = trajInit_.x();
        traj.pos.y() = trajInit_.y() - trajRadius_  * cos(2 * 3.14 * s) + trajRadius_ ;
        traj.pos.z() = trajInit_.z() - trajRadius_  * sin(2 * 3.14 * s);

        traj.vel.y() = trajRadius_  * 2 * 3.14 * s_d * sin(2 * 3.14 * s);
        traj.vel.z() = - trajRadius_  * 2 * 3.14 * s_d * cos(2 * 3.14 * s);

        traj.acc.y() = trajRadius_  * (2 * 3.14) * s_dd * sin(2 * 3.14 * s) + trajRadius_  * (2 * 3.14) * (2 * 3.14) * std::pow(s_d, 2) * cos(2 * 3.14 * s);
        traj.acc.z() = trajRadius_  * (2 * 3.14) * (2 * 3.14) * std::pow(s_d, 2) * sin(2 * 3.14 * s) - trajRadius_  * (2 * 3.14) * s_dd * cos(2 * 3.14 * s);
    }
    return traj;
}







