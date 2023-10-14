#include "controlMethod.H"

// * * * * * * * * * * * * Utility function  * * * * * * * * * * * * //



// * * * * * * * * * * * * Factory  * * * * * * * * * * * * //
const Foam::Enum
<
controlMethod::controlType
>
controlMethod::controlTypeNames
({
        {controlType::sailing, "sailing"},
        {controlType::turning, "turning"},
        {controlType::zigzag, "zigzag"},
        {controlType::coursekeeping, "coursekeeping"},
});

std::shared_ptr<controlMethod>
controlMethod::create(const dictionary &dict)
{
    const controlType type = controlTypeNames.get(dict.dictName());

    switch (type)
    {
    case sailing:
        return std::make_shared<sailingControl>(dict);
    case turning:
        return std::make_shared<turningControl>(dict);
    case zigzag:
        return std::make_shared<zigzagControl>(dict);
    case coursekeeping:
        return std::make_shared<coursekeepingControl>(dict);           
    default:
        FatalIOErrorInFunction(dict)
            << "    Unknown control method " << type
            << exit(FatalIOError);
        return nullptr;
    }
}

// * * * * * * * * * * * * Constructor  * * * * * * * * * * * * //
controlMethod::controlMethod(const dictionary &dict)
: 
  controlType_
  (
    controlTypeNames.get(dict.dictName())
  ),
  cStartTime_(dict.getOrDefault<scalar>("controllerStartTime", 0.)),
  cEndTime_(dict.getOrDefault<scalar>("controllerEndTime", 10000.)),
  outputSignal_(dict.getOrDefault<scalar>("controllerInitial", 0.))
{}

void controlMethod::write(Ostream &os) const
{
    os.writeEntry("maneuveringMode", controlTypeNames.get(controlType_));
}

scalar controlMethod::cStartTime() const
{
    return cStartTime_;
}

scalar controlMethod::cEndTime() const
{
    return cEndTime_;
}

scalar controlMethod::outputSignal() const
{
    return outputSignal_;
}

// * * * * * * * * * * * * turning Control  * * * * * * * * * * * * //
turningControl::turningControl(const dictionary &dict)
: 
    controlMethod(dict),
    cTarget_(dict.getOrDefault<scalar>("controllerYawAngle", 270.)),
    cMax_(dict.getOrDefault<scalar>("controllerRudder", 35.)),
    cRate_(dict.getOrDefault<scalar>("controllerRate", 5.)),
    outputSignal_(0.)
{}

scalar turningControl::calculate(scalar currentYaw, const scalar deltaT)
{
    Info<<nl<<"turningControl is running!!"<<nl;
    if(abs(outputSignal_) >= abs(cMax_))
    {
       outputSignal_ = cMax_;
       return outputSignal_;
    }
    outputSignal_ += cRate_*deltaT;
    return outputSignal_;
}

void turningControl::write(Ostream &os) const
{
    controlMethod::write(os);
    os.beginBlock("parameters");
    os.writeEntryIfDifferent("controllerTarget", 0.,  cTarget_);
    os.endBlock();
}

// * * * * * * * * * * * * zigzag Control  * * * * * * * * * * * * //
zigzagControl::zigzagControl(const dictionary &dict)
: 
    controlMethod(dict),
    cTarget_(dict.getOrDefault<scalar>("controllerYawAngle", 20.)),
    cMax_(dict.getOrDefault<scalar>("controllerRudder", 20.)),
    cRate_(dict.getOrDefault<scalar>("controllerRate", 5.)),
    outputSignal_(0.),
    oldYaw_(0.)
{
 
}

scalar zigzagControl::calculate(scalar currentYaw, const scalar deltaT)
{
    Info<<nl<<"zigzagControl is running!!"<<nl;
    if(oldYaw_ < abs(cTarget_) && currentYaw >= abs(cTarget_))
    {
       cRate_ = -1*abs(cRate_);
    }
       
    else if(oldYaw_ > -1*abs(cTarget_) && currentYaw <= -1*abs(cTarget_))
    {
       cRate_ = abs(cRate_);      
    } 
       
    outputSignal_ += cRate_*deltaT;
    
    if(outputSignal_ >= abs(cMax_))
    {
       outputSignal_ = abs(cMax_);    
    }
    
    else if(outputSignal_ <= -1*abs(cMax_))
    {
       outputSignal_ = -1*abs(cMax_);       
    }
          
    oldYaw_ = currentYaw;
    
    return outputSignal_;
}

void zigzagControl::write(Ostream &os) const
{
    controlMethod::write(os);
    os.beginBlock("parameters");
    os.writeEntryIfDifferent("controllerTarget", 0.,  cTarget_);
    os.endBlock();
}

// * * * * * * * * * * * * sailing Control  * * * * * * * * * * * * //
sailingControl::sailingControl(const dictionary &dict)
: 
    controlMethod(dict),
    P_(dict.getOrDefault<scalar>("controllerP", 1.)),
    I_(dict.getOrDefault<scalar>("controllerI", 1.)),
    D_(dict.getOrDefault<scalar>("controllerD", 0.)),
    cTarget_(dict.getOrDefault<scalar>("controllerTarget", 1.)),
    outputMax_(dict.getOrDefault<scalar>("controllerMax", 100.)),
    outputMin_(dict.getOrDefault<scalar>("controllerMin", 1.)),
    errorMax_(16.),
    integralErrorMax_(VGREAT),
    oldError_(0.),
    errorIntegral_(0.),
    outputSignal_(dict.getOrDefault<scalar>("controllerInitial", 0.))
{}

scalar sailingControl::calculate(scalar currentV, scalar deltaT)
{
    Info<<nl<<"sailingControl is running!!"<<nl;
    scalar error = cTarget_ - currentV;
    error = max(min(error, errorMax_), -errorMax_);  // Constain error according to specified errorMax
    errorIntegral_ += error * deltaT;
    errorIntegral_ = max(min(errorIntegral_, integralErrorMax_), -integralErrorMax_);
    const scalar errorDifferential = (error - oldError_);
    oldError_ = error;

    // Calculate increased output RPS value

    const scalar increasedOutputSignal = P_*error + I_*errorIntegral_ + D_*errorDifferential;
    outputSignal_ += increasedOutputSignal;
    
    // Return result within defined regulator saturation: outputMax_ and outputMin_
    return max(min(outputSignal_, outputMax_), outputMin_);
}

void sailingControl::write(Ostream &os) const
{
    controlMethod::write(os);
    os.beginBlock("parameters");
    os.writeEntry("Kp", P_);
    os.writeEntry("Ti", I_);
    os.writeEntry("Td", D_);
    os.writeEntryIfDifferent("outputMax", 1., outputMax_);
    os.writeEntryIfDifferent("outputMin", 0., outputMin_);
    os.writeEntryIfDifferent("errMax", VGREAT, errorMax_);
    os.writeEntryIfDifferent("errIntegMax", VGREAT, integralErrorMax_);
    os.endBlock();
}

// * * * * * * * * * * * * coursekeeping Control  * * * * * * * * * * * * //
coursekeepingControl::coursekeepingControl(const dictionary &dict)
: 
    controlMethod(dict),
    P_(dict.getOrDefault<scalar>("controllerP", 1.)),
    I_(dict.getOrDefault<scalar>("controllerI", 1.)),
    D_(dict.getOrDefault<scalar>("controllerD", 0.)),
    cTarget_(dict.getOrDefault<scalar>("controllerTarget", 0.)),
    cRate_(dict.getOrDefault<scalar>("controllerRate", 5.)),
    outputMax_(dict.getOrDefault<scalar>("controllerMax", 35.)),
    outputMin_(dict.getOrDefault<scalar>("controllerMin", -35.)),
    errorMax_(16.),
    integralErrorMax_(VGREAT),
    oldError_(0.),
    errorIntegral_(0.),
    outputSignal_(dict.getOrDefault<scalar>("controllerInitial", 0.))
{}

scalar coursekeepingControl::calculate(scalar currentV, scalar deltaT)
{
    Info<<nl<<"coursekeepingControl is running!!"<<nl;
    scalar error = cTarget_ - currentV;
    error = max(min(error, errorMax_), -errorMax_);  // Constain error according to specified errorMax
    errorIntegral_ += error * deltaT;
    errorIntegral_ = max(min(errorIntegral_, integralErrorMax_), -integralErrorMax_);
    const scalar errorDifferential = (error - oldError_);
    oldError_ = error;

    // Calculate increased output RPS value

    scalar increasedOutputSignal = P_*error + I_*errorIntegral_ + D_*errorDifferential;
    const scalar deltaMax = abs(deltaT*cRate_);
    if(abs(increasedOutputSignal)>= deltaMax)
    {
      increasedOutputSignal = increasedOutputSignal/(abs(increasedOutputSignal)+VSMALL)*deltaMax;
    }
    outputSignal_ += increasedOutputSignal;
    
    // Return result within defined regulator saturation: outputMax_ and outputMin_
    return max(min(outputSignal_, outputMax_), outputMin_);
}

void coursekeepingControl::write(Ostream &os) const
{
    controlMethod::write(os);
    os.beginBlock("parameters");
    os.writeEntry("Kp", P_);
    os.writeEntry("Ti", I_);
    os.writeEntry("Td", D_);
    os.writeEntryIfDifferent("outputMax", 1., outputMax_);
    os.writeEntryIfDifferent("outputMin", 0., outputMin_);
    os.writeEntryIfDifferent("errMax", VGREAT, errorMax_);
    os.writeEntryIfDifferent("errIntegMax", VGREAT, integralErrorMax_);
    os.endBlock();
}

