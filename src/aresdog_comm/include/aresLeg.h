
#include "common/mathTypes.h"


class QuadrupedLeg{
public:
    QuadrupedLeg(int legID, float hipLinkLength,float kneeLinkLength, Vec3 pHip2B);
    ~QuadrupedLeg();
    Vec3 calcPEe2H(Vec3 q);
    Vec3 calcPEe2B(Vec3 q);
    Vec3 calcQ(Vec3 pEe, FrameType frame);
    Vec3 calcQd(Vec3 q, Vec3 vEe);
    Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);

protected:
    float Q_ik(float px, float py);
    float _sideSign;
    const float _hipLinkLength, _kneeLinkLength;
    const float pHip2B;
};