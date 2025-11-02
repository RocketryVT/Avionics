import SwiftUI

struct CompassView: View {
    var headingToRocket: Double?
    var deviceHeading: Double
    var relativeHeadingToRocket: Double?
    var distance: Double?
    
    var body: some View {
        VStack {
            ZStack {
                // Compass circle
                Circle()
                    .stroke(Color.secondary.opacity(0.5), lineWidth: 1)
                    .background(Circle().fill(Color.secondary.opacity(0.1)))
                    .frame(width: 70, height: 70)
                
                // North indicator - rotates opposite to device heading to stay pointed north
                Text("N")
                    .font(.system(size: 12, weight: .bold))
                    .foregroundColor(.secondary)
                    .offset(y: -30)
                    .rotationEffect(Angle(degrees: -deviceHeading))
                
                // Arrow pointing to rocket - uses the relative heading
                if let relativeBearing = relativeHeadingToRocket {
                    Image(systemName: "location.north.fill")
                        .font(.system(size: 24))
                        .foregroundColor(.red)
                        .rotationEffect(Angle(degrees: relativeBearing))
                } else {
                    // Show question mark if no heading available
                    Image(systemName: "questionmark")
                        .font(.system(size: 24))
                        .foregroundColor(.secondary)
                }
                
                // Distance label if available
                if let distance = distance {
                    Text(String(format: "%.1f m", distance))
                        .font(.system(size: 10))
                        .foregroundColor(.secondary)
                        .offset(y: 20)
                }
            }
        }
        .frame(width: 80, height: 80)
        .background(Color.white.opacity(0.7))
        .cornerRadius(40)
        .shadow(radius: 2)
    }
}