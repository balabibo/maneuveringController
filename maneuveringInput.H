/*---------------------------------------------------------------------------*\
Class
    Foam::maneuveringInput

Description
    Part of the movingController which is used to read sailing velocity or 
yaw angle of parent level mesh.

\*---------------------------------------------------------------------------*/

#ifndef Foam_maneuveringInput_H
#define Foam_maneuveringInput_H

#include "fvCFD.H"

class maneuveringInput
{
public:
    virtual ~maneuveringInput() = default;
    maneuveringInput() = delete;

    // Initilize from mesh and dictionary
    maneuveringInput(const dictionary &dict, const uniformDimensionedVectorField& velocityAndyaw);
    //maneuveringInput(const fvMesh &mesh, const Time& runTime);
           
    scalar sailingVelocity() const;

    scalar yawAngle() const;

    word inputName() const;

    virtual scalar input() const = 0;

    static std::shared_ptr<maneuveringInput> create(const dictionary &dict, const uniformDimensionedVectorField& velocityAndyaw);

    enum inputType
    {
       sailing,
       turning,
       zigzag,
       coursekeeping
    };

    static const Enum<inputType> inputTypeNames;
    
    void update();

protected:
      
    // the sailingVelocity of the ship/rigid 
    scalar sailingVelocity_; 
    
    // the yawAngle of the ship/rigid 
    scalar yawAngle_;

    const word inputName_;
    
    const uniformDimensionedVectorField& velocityAndyaw_;
    
    
    
};

class sailingInput : public maneuveringInput
{
public:
    sailingInput() = delete;
    sailingInput(const dictionary &dict, const uniformDimensionedVectorField& velocityAndyaw);
    scalar input() const override;
   
};

class yawInput : public maneuveringInput
{
public:
    yawInput() = delete;
    yawInput(const dictionary &dict, const uniformDimensionedVectorField& velocityAndyaw);
    scalar input() const override;
   
};


#endif
