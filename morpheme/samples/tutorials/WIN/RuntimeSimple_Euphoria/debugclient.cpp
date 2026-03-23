#include "NMPlatform/NMPrioritiesLogger.h"
#include "NMPlatform/NMSocket.h"
#include "morpheme/mrDebugManager.h"
#include "morpheme/mrDebugClient.h"
#include "morpheme/mrManager.h"

class OpenGL_DebugClient : public MR::DebugClient
{
public:
    // -------------- DebugClient --------------
    void Init();


    // -------------- DebugClient --------------
    void clearDebugDraw() NM_OVERRIDE;
    void drawPoint(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& point,
        float               radius,
        NMP::Colour         colour) NM_OVERRIDE;
    void drawSphere(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Matrix34& point,
        float               radius,
        NMP::Colour         colour) NM_OVERRIDE;
    // Draw a basic line from start to end.
    void drawLine(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& start,
        const NMP::Vector3& end,
        NMP::Colour         colour) NM_OVERRIDE;
    void drawPolyLine(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        uint32_t            numVertices,       ///< Min of 2 and numLines = numVetices - 1;
        const NMP::Vector3* vertices,          ///< Array of vertices between lines.
        NMP::Colour         colour) NM_OVERRIDE;
    void drawVector(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        MR::VectorType          type,
        const NMP::Vector3& start,
        const NMP::Vector3& offset,
        NMP::Colour         colour) NM_OVERRIDE;
    void drawArrowHead(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& startPos,          ///< Position of arrow point (if not a delta).
        const NMP::Vector3& direction,         ///< Arrow direction.
        const NMP::Vector3& tangent,           ///< Generally at 90 degrees to the direction. Defines the width of the arrow.
        NMP::Colour         colour,
        bool                hasMass,           ///< If true draws solid arrows, else draw line arrows.
        bool                isDelta) NM_OVERRIDE;          ///< Controls how arrow heads are drawn (conventionally, inverted, or as a line cap)
    void drawTwistArc(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& pos,               ///< Center of arc.
        const NMP::Vector3& primaryDir,        ///< Normal of the plane which the arc lies on (Not necessarily normalised).
        const NMP::Vector3& dir,               ///<
        const NMP::Vector3& dir2,              ///<
        NMP::Colour         colour,
        bool                doubleArrowHead,   ///< Draw an arrow at both ends of arc.
        bool                hasMass,           ///< Arrow heads are drawn as solid triangles.
        bool                isDelta) NM_OVERRIDE;          ///< Controls how arrow heads are drawn (conventionally, inverted, or as a line cap)
    void drawPlane(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& point,
        const NMP::Vector3& normal,
        float               radius,
        NMP::Colour         colour,
        float               normalScale) NM_OVERRIDE;
    void drawTriangle(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& v1,                ///< 1-2-3 should form a clockwise face.
        const NMP::Vector3& v2,
        const NMP::Vector3& v3,
        NMP::Colour         colour) NM_OVERRIDE;
    void drawMatrix(
        uint32_t             sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID           sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount       sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex        sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Matrix34& matrix,
        float                scale,
        uint8_t              alpha) NM_OVERRIDE;
    void drawNonUniformMatrix(
        uint32_t             sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID           sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount       sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex        sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Matrix34& matrix,
        float                scale,
        uint8_t               alpha) NM_OVERRIDE;
    void drawConeAndDial(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& pos,
        const NMP::Vector3& dir,
        float               angle,
        float               size,
        const NMP::Vector3& dialDirection,
        NMP::Colour         colour) NM_OVERRIDE;
    void drawContactPointSimple(
        uint32_t            sourceInstanceID,       ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,           ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,          ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,            ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,        ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& point,                  ///< Contact point.
        float               forceMagnitudeSquared) NM_OVERRIDE; ///< Force magnitude squared at contact point. 
    void drawContactPointDetailed(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& point,             ///< Contact point.
        const NMP::Vector3& normal,            ///< Contact normal.
        const NMP::Vector3& force,             ///< Force at contact point. 
        const NMP::Vector3& actor0Pos,         ///< Actor 0 root position.
        const NMP::Vector3& actor1Pos) NM_OVERRIDE;        ///< Actor 1 root position.
    void drawCharacterRoot(
        uint32_t             sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID           sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount       sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex        sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Matrix34& characterControllerRoot) NM_OVERRIDE;
    void drawBox(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& focalCentre,
        const NMP::Vector3& focalRadii,
        NMP::Colour         colour) NM_OVERRIDE;
    void drawBBox(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Matrix34& tm,
        const NMP::Vector3& focalRadii,
        NMP::Colour         colour) NM_OVERRIDE;
    void drawSphereSweep(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& position,
        const NMP::Vector3& motion,
        float               radius,
        NMP::Colour         colour) NM_OVERRIDE;
    void drawText(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& position,          ///< Bottom left hand corner of string.
        const char* text,
        NMP::Colour         colour) NM_OVERRIDE;
    void drawEnvironmentPatch(
        uint32_t            sourceInstanceID,  ///< Unique Character/Network instance identifier.
        MR::NodeID          sourceNodeID,      ///< INVALID_NODE_ID if its not from a specific node.
        const char* sourceTagName,     ///< NULL if not from a specific module.
        MR::FrameCount      sourceFrame,       ///< What frame this data is from. VALID_FRAME_ANY_FRAME if not specific to a frame.
        MR::LimbIndex       sourceLimbIndex,   ///< The limb index. INVALID_LIMB_INDEX if not for any limb and in all morpheme data.
        const NMP::Vector3& position,
        const NMP::Vector3& normal,
        const float         size,
        NMP::Colour         colour) NM_OVERRIDE;



};




void OpenGL_DebugClient::clearDebugDraw()
{

}
void OpenGL_DebugClient::drawPoint( uint32_t sourceInstanceID, 
    MR::NodeID sourceNodeID, const char* sourceTagName, 
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex, 
    const NMP::Vector3& point, float radius, 
    NMP::Colour colour)
{

}
void OpenGL_DebugClient::drawSphere( uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex, 
    const NMP::Matrix34& point, float radius, 
    NMP::Colour colour)
{

}
// Draw a basic line from start to end.
void OpenGL_DebugClient::drawLine(uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
    const NMP::Vector3& start, const NMP::Vector3& end,
    NMP::Colour colour)
{

}
void OpenGL_DebugClient::drawPolyLine(uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
    uint32_t numVertices, const NMP::Vector3* vertices,
    NMP::Colour colour)
{

}
void OpenGL_DebugClient::drawVector(uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
    MR::VectorType type, const NMP::Vector3& start,
    const NMP::Vector3& offset, NMP::Colour colour)
{

}
void OpenGL_DebugClient::drawArrowHead( uint32_t sourceInstanceID, 
        MR::NodeID sourceNodeID, const char* sourceTagName, 
        MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex, 
        const NMP::Vector3& startPos, const NMP::Vector3& direction, 
        const NMP::Vector3& tangent, NMP::Colour colour, 
        bool hasMass, bool isDelta)
{

}
void OpenGL_DebugClient::drawTwistArc( uint32_t sourceInstanceID, 
        MR::NodeID sourceNodeID, const char* sourceTagName, 
        MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex, 
        const NMP::Vector3& pos, const NMP::Vector3& primaryDir, 
        const NMP::Vector3& dir, const NMP::Vector3& dir2, 
        NMP::Colour colour, bool doubleArrowHead, 
        bool hasMass, bool isDelta)
{

}
void OpenGL_DebugClient::drawPlane(uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
    const NMP::Vector3& point, const NMP::Vector3& normal,
    float radius, NMP::Colour colour,
    float normalScale)
{

}
void OpenGL_DebugClient::drawTriangle(uint32_t sourceInstanceID,
        MR::NodeID sourceNodeID, const char* sourceTagName,
        MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
        const NMP::Vector3& v1, const NMP::Vector3& v2,
        const NMP::Vector3& v3, NMP::Colour colour)
{

}
void OpenGL_DebugClient::drawMatrix(uint32_t sourceInstanceID,
        MR::NodeID sourceNodeID, const char* sourceTagName,
        MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
        const NMP::Matrix34& matrix, float scale,
        uint8_t alpha)
{

}
void OpenGL_DebugClient::drawNonUniformMatrix(uint32_t sourceInstanceID,
        MR::NodeID sourceNodeID, const char* sourceTagName,
        MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
        const NMP::Matrix34& matrix, float scale,
        uint8_t alpha)
{

}
void OpenGL_DebugClient::drawConeAndDial(uint32_t sourceInstanceID,
        MR::NodeID sourceNodeID, const char* sourceTagName,
        MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
        const NMP::Vector3& pos, const NMP::Vector3& dir,
        float angle, float size,
        const NMP::Vector3& dialDirection, NMP::Colour colour)
{

}
void OpenGL_DebugClient::drawContactPointSimple(uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
    const NMP::Vector3& point, float forceMagnitudeSquared)
{

}
void OpenGL_DebugClient::drawContactPointDetailed(uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
    const NMP::Vector3& point, const NMP::Vector3& normal,
    const NMP::Vector3& force, const NMP::Vector3& actor0Pos,
    const NMP::Vector3& actor1Pos)
{

}
void OpenGL_DebugClient::drawCharacterRoot(uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
    const NMP::Matrix34& characterControllerRoot)
{

}
void OpenGL_DebugClient::drawBox( uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame,
    MR::LimbIndex sourceLimbIndex,
    const NMP::Vector3& focalCentre,
    const NMP::Vector3& focalRadii,
    NMP::Colour colour)
{

}
void OpenGL_DebugClient::drawBBox(uint32_t sourceInstanceID, 
    MR::NodeID sourceNodeID, const char* sourceTagName, 
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex, 
    const NMP::Matrix34& tm, const NMP::Vector3& focalRadii, 
    NMP::Colour colour)
{

}
void OpenGL_DebugClient::drawSphereSweep(uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
    const NMP::Vector3& position, const NMP::Vector3& motion,
    float radius, NMP::Colour colour)
{

}
void OpenGL_DebugClient::drawText( uint32_t sourceInstanceID,
    MR::NodeID sourceNodeID, const char* sourceTagName,
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex,
    const NMP::Vector3& position, const char* text,
    NMP::Colour colour)
{

}
void OpenGL_DebugClient::drawEnvironmentPatch( uint32_t sourceInstanceID, 
    MR::NodeID sourceNodeID, const char* sourceTagName, 
    MR::FrameCount sourceFrame, MR::LimbIndex sourceLimbIndex, 
    const NMP::Vector3& position, const NMP::Vector3& normal, 
    const float size, NMP::Colour colour)
{

}
