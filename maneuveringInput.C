#include "maneuveringInput.H"

const Foam::Enum
<
maneuveringInput::inputType
>
maneuveringInput::inputTypeNames
({
        {inputType::sailing, "sailing"},
        {inputType::turning, "turning"},
        {inputType::zigzag, "zigzag"},
        {inputType::coursekeeping, "coursekeeping"},
});

std::shared_ptr<maneuveringInput>
maneuveringInput::create(const dictionary &dict, const uniformDimensionedVectorField& velocityAndyaw)
{
    const inputType type = inputTypeNames.get(dict.dictName());

    switch (type)
    {
    case sailing:
        return std::make_shared<sailingInput>(dict, velocityAndyaw);
    case turning:
        return std::make_shared<yawInput>(dict, velocityAndyaw);
    case zigzag:
        return std::make_shared<yawInput>(dict, velocityAndyaw);     
    case coursekeeping:
        return std::make_shared<yawInput>(dict, velocityAndyaw);          
    default:
        FatalIOErrorInFunction(dict)
            << "    Unknown control method " << type
            << exit(FatalIOError);
        return nullptr;
    }
}



// * * * * * * * * * * * * Base maneuveringInput  * * * * * * * * * * * * //

maneuveringInput::maneuveringInput(const dictionary &dict, const uniformDimensionedVectorField& velocityAndyaw)
:
    sailingVelocity_(velocityAndyaw.value().x()),
    yawAngle_(velocityAndyaw.value().y()),
    inputName_(dict.dictName()),
    velocityAndyaw_(velocityAndyaw)
{
    
    Info<<nl<<"sailingVelocity: "<<this->sailingVelocity()<<nl<<endl
        <<nl<<"yawAngle: "<<this->yawAngle()<<nl<<endl;
}

scalar maneuveringInput::sailingVelocity() const
{
    return sailingVelocity_;
}

scalar maneuveringInput::yawAngle() const
{
    return yawAngle_;
}

word maneuveringInput::inputName() const
{
    return inputName_;
}

void maneuveringInput::update()
{
    sailingVelocity_ = velocityAndyaw_.value().x();
    yawAngle_ = velocityAndyaw_.value().y();
}

// * * * * * * * * * * * * sailingInput  * * * * * * * * * * * * //
sailingInput::sailingInput(const dictionary &dict, const uniformDimensionedVectorField& velocityAndyaw)
:
    maneuveringInput(dict, velocityAndyaw)
{

}

scalar sailingInput::input() const
{

   return this->sailingVelocity();

}

// * * * * * * * * * * * * yawInput  * * * * * * * * * * * * //
yawInput::yawInput(const dictionary &dict, const uniformDimensionedVectorField& velocityAndyaw)
:
    maneuveringInput(dict, velocityAndyaw)
{

}

scalar yawInput::input() const
{

   return this->yawAngle();

}