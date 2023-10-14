/*---------------------------------------------------------------------------*\
Class
    Foam::controlMethod

Description
    controlMethod is a base class of the maneuveringOutput, which provides control algorithm for the maneuvering motions.


\*---------------------------------------------------------------------------*/
#ifndef controlMethod_H
#define controlMethod_H

#include "fvCFD.H"

class controlMethod
{
public:
    virtual ~controlMethod() = default;
    controlMethod() = default;
    controlMethod(const dictionary &);

    enum controlType
    {
        sailing, // on/off control
        turning,
        zigzag,
        coursekeeping,
    };
    static const Enum<controlType> controlTypeNames;

    //- controlMethod factory
    static std::shared_ptr<controlMethod> create(const dictionary &);    

    //- Calculate output signal from current process value, target value
    // and current time delta
    //virtual scalar calculate(scalar , scalar , const scalar ) = 0;


    //- Calculate output signal from current process value and current time delta
    virtual scalar calculate(scalar, const scalar ) = 0;

    //- Write to runtime dict
    virtual void write(Ostream &) const;
    
    //- refer to start Time
    scalar cStartTime() const;
    
    //- refer to end Time
    scalar cEndTime() const;

    //- refer to output value
    scalar outputSignal() const;
    
protected:
    const enum controlType controlType_;
    
    // Start time of controller
    scalar cStartTime_;
    
    // End time of controller
    scalar cEndTime_;

    //output value
    scalar outputSignal_;   
};

class turningControl : public controlMethod
{
public:
    turningControl() = default;
    turningControl(const dictionary &);
    
    // currentYaw refers to current Yaw angle

    scalar calculate(scalar currentYaw, const scalar deltaT);
    void write(Ostream &) const override;

private:

    // target yaw angle
    scalar cTarget_;
    
    // maximum controller value 
    scalar cMax_;
    
    // controller rate  
    scalar cRate_;
    
    // Preserve old output signal for histeresis correction
    scalar outputSignal_;

};

class zigzagControl : public controlMethod
{
public:
    zigzagControl() = default;
    zigzagControl(const dictionary &);
    
    // currentYaw refers to current yaw angle.
    scalar calculate(scalar currentYaw, scalar deltaT);
    
    void write(Ostream &) const override;

private:
    
    // target yaw angle
    scalar cTarget_;
    
    // maximum controller value 
    scalar cMax_;
    
    // controller rate  
    scalar cRate_;
    
    // output rudder angle
    scalar outputSignal_;
    
    // restore the yaw angle of the previous time-step.
    scalar oldYaw_; 
    
};

class sailingControl : public controlMethod
{
public:
    sailingControl() = default;
    sailingControl(const dictionary &);
   
    // currentV refers to current sailing velocity
    scalar calculate(scalar currentV, scalar deltaT);
    void write(Ostream &) const override;


private:
    scalar P_;                     // Proportional control term
    scalar I_;                     // Integral control term
    scalar D_;                     // Differential control term
    scalar cTarget_;                // target sailing velocity
    scalar outputMax_;              // Max output signal (saturation)
    scalar outputMin_;              // Min output signal
    scalar errorMax_;               // Constains received absolute error value
    scalar integralErrorMax_;       // Constains built-up integral error
    scalar oldError_;               // Old error
    scalar errorIntegral_;          // Error integral w.r.t. time
    scalar outputSignal_;           // output RPS
};

class coursekeepingControl : public controlMethod
{
public:
    coursekeepingControl() = default;
    coursekeepingControl(const dictionary &);
   
    // currentV refers to current sailing velocity
    scalar calculate(scalar currentV, scalar deltaT);
    void write(Ostream &) const override;


private:
    scalar P_;                     // Proportional control term
    scalar I_;                     // Integral control term
    scalar D_;                     // Differential control term
    scalar cTarget_;                // target sailing velocity
    scalar cRate_;                  // controller rate
    scalar outputMax_;              // Max output signal (saturation)
    scalar outputMin_;              // Min output signal  
    scalar errorMax_;               // Constains received absolute error value
    scalar integralErrorMax_;       // Constains built-up integral error
    scalar oldError_;               // Old error
    scalar errorIntegral_;          // Error integral w.r.t. time
    scalar outputSignal_;           // output RPS
};
#endif
