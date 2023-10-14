#include "maneuveringOutput.H"



// * * * * * * * * * * * * Base maneuveringOutput  * * * * * * * * * * * * //


// * * * * * * * * * * * * * * * * Constructor * * * * * * * * * * * * * * * //

maneuveringOutput::maneuveringOutput(const fvMesh &mesh, const dictionary &dict, const uniformDimensionedVectorField& velocityAndyaw)
: 
    mesh_(mesh),
    sensor_(maneuveringInput::create(dict, velocityAndyaw)),
    controlMethod_(controlMethod::create(dict)),
    timeIndex_(mesh.time().timeIndex())
{}


// * * * * * * * * * * * * Public Member Functions  * * * * * * * * * * * * *//

scalar maneuveringOutput::output()
{
    // Get time data
    const scalar deltaT = mesh_.time().deltaTValue();
    const scalar t = mesh_.time().timeOutputValue();
    if(t <  controlMethod_->cStartTime() || t > controlMethod_->cEndTime())
    {
 
       return controlMethod_->outputSignal(); 

    }

    // Update the old-time quantities
    if (timeIndex_ != mesh_.time().timeIndex())
    {
        timeIndex_ = mesh_.time().timeIndex();
    }

    sensor_->update();
    // Get the target patch average field value
    const scalar InputValue = sensor_->input();
    
    const scalar outputSignal = controlMethod_->calculate(InputValue, deltaT);

    //Info << "maneuveringOutput: targetValue = " << targetValue_->value(t) << endl;
    Info << "maneuveringOutput: currentValue = " << InputValue << endl;
    //Info << "maneuveringOutput: error = " << targetValue_->value(t) - sensorValue  << endl;
    Info << "maneuveringOutput: outputSignal = " << outputSignal << endl;

    return outputSignal;
}

void maneuveringOutput::write(Ostream& os, const word dictName) const
{
    os.beginBlock(dictName);
    //targetValue_->writeData(os);
   // os.writeEntry("field", sensor_->fieldName());

    controlMethod_->write(os);

    //os.beginBlock("sensor");
    //sensor_->write(os);
    //os.endBlock();

    os.endBlock();
}








