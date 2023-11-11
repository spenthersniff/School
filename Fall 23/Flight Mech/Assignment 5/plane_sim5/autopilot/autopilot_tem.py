import numpy as np
import compute_gains.auto_gains as gains



class Autopilot():
    def __init__(self, dt, altitude_take_off_zone, altitude_hold_zone):
        self.dt = dt
        self.altitude_take_off_zone = altitude_take_off_zone
        self.altitude_hold_zone = altitude_hold_zone
        self.initialize_integrator = 1.0
        self.altitude_state = 0.0

    def autopilot(self, u):
        t=u[0]
        phi=u[1]
        theta=u[2]
        chi=u[3]
        p=u[4]
        q=u[5]
        r=u[6]
        Va=u[7]
        h=u[8]
        Va_c=u[9]
        h_c=u[10]
        chi_c=u[11]
        
        #Lateral Autopilot
        if t==0:
            delta_r=0
            phi_c=self.course_hold(chi_c,chi,r,1,self.dt)
            delta_a=self.roll_hold(phi_c,phi,p,1,self.dt)
        else:
            delta_r=0
            phi_c=self.course_hold(chi_c,chi,r,0,self.dt)
            delta_a=self.roll_hold(phi_c,phi,p,0,self.dt)
        
        #Longitudinal Autopilot
        if t==0:
            if h<=self.altitude_take_off_zone:
                self.altitude_state=0
            elif h<=h_c-self.altitude_hold_zone:
                self.altitude_state=1
            elif h>=h_c+self.altitude_hold_zone:
                self.altitude_state=2
            else:
                self.altitude_state=3
            self.initialize_integrator=1
        
        if self.altitude_state==0:
            delta_t=1
            theta_c=10*(np.pi/180)
            if h>=self.altitude_take_off_zone:
                self.altitude_state=1
                self.initialize_integrator=1
            else:
                self.initialize_integrator=0
        elif self.altitude_state==1:
            delta_t=1
            theta_c=self.airspeed_hold_pitch(Va_c,Va,self.initialize_integrator,self.dt)
            
            if h>=h_c-self.altitude_take_off_zone:
                self.altitude_state=3
                self.initialize_integrator=1
            elif h<=self.altitude_take_off_zone:
                self.altitude_state=0
                self.initialize_integrator=1
            else:
                self.initialize_integrator=0
        elif self.altitude_state==2:
            delta_t=0
            theta_c=self.airspeed_hold_pitch(Va_c,Va,self.initialize_integrator,self.dt)
            if h<=h_c+self.altitude_hold_zone:
                self.altitude_state=3
                self.initialize_integrator=1
            else:
                self.initialize_integrator=0
        elif self.altitude_state==3:
            delta_t=self.airspeed_hold_throttle(Va_c,Va,self.initialize_integrator,self.dt)
            theta_c=self.altitude_hold(h_c,h,self.initialize_integrator,self.dt)
            if h<=h_c-self.altitude_hold_zone:
                self.altitude_state=1
                self.intiailize_integrator=1
            elif h>=h_c+self.altitude_hold_zone:
                self.altitude_state=2
                self.initialize_integrator=1
            else:
                self.initialize_integrator=0
                
        if t==0:
            delta_e=self.pitch_hold(theta_c,theta,q,1,self.dt)
        else:
            delta_e=self.pitch_hold(theta_c,theta,q,0,self.dt)
        
        # return (delta_e,delta_a,delta_r,delta_t,phi_c,theta_c,chi_c,altitude_state)
        return (delta_e,delta_a,delta_r,delta_t)
    
    def roll_hold(self, phi_c,phi,p,flag,dt):
        
        limit1=np.radians(45)
        limit2=-np.radians(45)
        
        kp=gains.kp_roll
        kd=gains.kd_roll
        ki=gains.ki_roll
        
        if flag==1:
            self.roll_integrator=0
            self.roll_differentiator=0
            self.roll_error_d1=0
        
        error=phi_c-phi
        self.roll_integrator=self.roll_integrator+(dt/2)*(error+self.roll_error_d1)
        self.roll_differentiator=p
        self.roll_error_d1=error
        
        u=kp*error+ki*self.roll_integrator+kd*self.roll_differentiator

        
        u_sat=self.sat(u,limit1,limit2)
        if ki!=0:
            self.roll_integrator=self.roll_integrator+dt/ki*(u_sat-u)
            
        return u_sat
    
    def pitch_hold(self, theta_c,theta,q,flag,dt):
        
        limit1=np.radians(45)
        limit2=-np.radians(45)
        
        kp=gains.kp_pitch
        kd=gains.kd_pitch
        ki=gains.ki_pitch
        
        if flag==1:
            self.pitch_integrator=0
            self.pitch_differentiator=0
            self.pitch_error_d1=0
        
        error=theta_c-theta
        self.pitch_integrator=self.pitch_integrator+(dt/2)*(error+self.pitch_error_d1)
        self.pitch_differentiator=q
        self.pitch_error_d1=error
        
        u=kp*error+ki*self.pitch_integrator+kd*self.pitch_differentiator
        
        u_sat=self.sat(u,limit1,limit2)
        if ki!=0:
            self.pitch_integrator=self.pitch_integrator+dt/ki*(u_sat-u)
            
        return u_sat
    
    def course_hold(self, chi_c,chi,r,flag,dt):
        
        limit1=np.radians(45)
        limit2=-np.radians(45)

        kp=gains.kp_course
        kd=gains.kd_course
        ki=gains.ki_course
        
        if flag==1:
            self.course_integrator=0
            self.course_differentiator=0
            self.course_error_d1=0
        
        error=chi_c-chi
        self.course_integrator=self.course_integrator+(dt/2)*(error+self.course_error_d1)
        self.course_differentiator=r
        self.course_error_d1=error
        
        u=kp*error+ki*self.course_integrator+kd*self.course_differentiator
        

        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.course_integrator = self.course_integrator + dt/ki*(u_sat - u)
        
        return u_sat

    def airspeed_hold_pitch(self, Va_c,Va,flag,dt):
       
        limit1=np.radians(45)
        limit2=-np.radians(45)

        kp=gains.kp_airspeed
        kd=gains.kd_airspeed
        ki=gains.ki_airspeed
        tau=5
        
        if flag==1:
            self.ahp_integrator=0
            self.ahp_differentiator=0
            self.ahp_error_d1=0
        
        error=Va_c-Va
        self.ahp_integrator=self.ahp_integrator+(dt/2)*(error+self.ahp_error_d1)
        self.ahp_differentiator=(2*tau - dt)/(2*tau + dt)*self.ahp_differentiator + 2/(2*tau + dt)*(error - self.ahp_error_d1)
        self.ahp_error_d1=error
        
        u=kp*error+ki*self.ahp_integrator+kd*self.ahp_differentiator
        
        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.ahp_integrator = self.ahp_integrator + dt/ki*(u_sat - u)
       
        return u_sat
    
    def airspeed_hold_throttle(self, Va_c,Va,flag,dt):
        
        limit1=1.0
        limit2=0.0
        
        kp=gains.kp_throttle
        kd=gains.kd_throttle
        ki=gains.ki_throttle
        tau=5

        if flag==1:
            self.aht_integrator=0
            self.aht_differentiator=0
            self.aht_error_d1=0
        
        error=Va-Va_c
        self.aht_integrator=self.aht_integrator+(dt/2)*(error+self.aht_error_d1)
        self.aht_differentiator=(2*tau - dt)/(2*tau + dt)*self.aht_differentiator + 2/(2*tau + dt)*(error - self.aht_error_d1)
        self.aht_error_d1=error
        
        u=kp*error+ki*self.aht_integrator+kd*self.aht_differentiator
        
        u_sat=self.sat(u,limit1,limit2)
        if ki!=0:
            self.pitch_integrator=self.pitch_integrator+dt/ki*(u_sat-u)
            
        return u_sat
    
    def altitude_hold(self, h_c,h,flag,dt):
       
        limit1=np.radians(45)
        limit2=-np.radians(45)

        kp=gains.kp_altitude
        kd=gains.kd_altitude
        ki=gains.ki_altitude
        tau=5
        
        if flag==1:
            self.altitude_integrator=0
            self.altitude_differentiator=0
            self.altitude_error_d1=0
        
        error=h_c-h
        self.altitude_integrator=self.altitude_integrator+(dt/2)*(error+self.altitude_error_d1)
        self.altitude_differentiator=(2*tau - dt)/(2*tau + dt)*self.altitude_differentiator + 2/(2*tau + dt)*(error - self.altitude_error_d1)
        self.altitude_error_d1=error
        
        u=kp*error+ki*self.altitude_integrator+kd*self.altitude_differentiator
        
        u_sat = self.sat(u, limit1, limit2)
        if ki != 0:
            self.altitude_integrator = self.altitude_integrator + dt/ki*(u_sat - u)

        return u_sat

    def sat(self,inn, up_limit, low_limit):
        if inn > up_limit:
            out = up_limit
        elif inn < low_limit:
            out = low_limit
        else:
            out = inn
        return out
    

