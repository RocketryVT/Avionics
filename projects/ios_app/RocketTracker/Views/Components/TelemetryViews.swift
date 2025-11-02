//
//  TelemetryViews.swift
//  RocketTracker
//
//  Created by Gregory Wainer on 5/14/25.
//

import SwiftUI

struct TelemetryDataView: View {
    @ObservedObject var presenter: MainPresenter
    @State private var showLogsSheet = false
    @State private var selectedDeviceTab: UInt32?
    
    var body: some View {
        VStack {
            // Device selector tabs (if multiple devices)
            if presenter.getAvailableDeviceIDs().count > 1 {
                ScrollView(.horizontal, showsIndicators: false) {
                    HStack {
                        Button(action: {
                            selectedDeviceTab = nil
                            presenter.selectDevice(nil)
                        }) {
                            Text("All Devices")
                                .padding(.horizontal, 12)
                                .padding(.vertical, 12)
                                .background((selectedDeviceTab == nil) ? Color.blue : Color.gray.opacity(0.2))
                                .cornerRadius(16)
                                .foregroundColor((selectedDeviceTab == nil) ? .white : .primary)
                        }
                        
                        ForEach(presenter.getAvailableDeviceIDs(), id: \.self) { deviceID in
                            Button(action: {
                                selectedDeviceTab = deviceID
                                presenter.selectDevice(deviceID)
                            }) {
                                Text("Device \(deviceID)")
                                    .padding(.horizontal, 12)
                                    .padding(.vertical, 12)
                                    .background((selectedDeviceTab == deviceID) ? Color.blue : Color.gray.opacity(0.2))
                                    .cornerRadius(16)
                                    .foregroundColor((selectedDeviceTab == deviceID) ? .white : .primary)
                            }
                        }
                    }
                    .padding(.horizontal)
                }
            }
            
            // Telemetry data display
            ScrollView {
                if selectedDeviceTab == nil && presenter.getAvailableDeviceIDs().count > 1 {
                    // Show data for all devices
                    ForEach(presenter.getAvailableDeviceIDs(), id: \.self) { deviceID in
                        if let telemetry = presenter.getTelemetryData(for: deviceID) {
                            VStack(alignment: .leading) {                                
                                DeviceTelemetryView(telemetry: telemetry)
                                    .padding()
                                
                                Divider()
                            }
                        }
                    }
                } else if let deviceID = selectedDeviceTab, let telemetry = presenter.getTelemetryData(for: deviceID) {
                    // Show data for selected device
                    DeviceTelemetryView(telemetry: telemetry)
                        .padding()
                } else if let telemetry = presenter.telemetryData {
                    // Fallback to single device mode
                    DeviceTelemetryView(telemetry: telemetry)
                        .padding()
                } else {
                    
                }
            }
        }
        .navigationTitle("Rocket Telemetry")
        .toolbar {
            ToolbarItem(placement: .navigationBarLeading) {
                Button(action: {
                    showLogsSheet = true
                }) {
                    Label("Logs", systemImage: "list.bullet.clipboard")
                }
            }
        }
        .sheet(isPresented: $showLogsSheet) {
            TelemetryLogBrowser(presenter: presenter)
        }
    }
}

struct TelemetryLogBrowser: View {
    @ObservedObject var presenter: MainPresenter
    @State private var selectedDate: Date?
    @State private var recordsForDate: [TelemetryRecord] = []
    @State private var showingDataDetail = false
    @State private var logDates: [Date] = []
    @State private var isLoading = false
    @State private var showingDeleteAlert = false
    @State private var dateToDelete: Date?
    @State private var selectedDeviceFilter: UInt32?
    @State private var availableDevices: [UInt32] = []
    @Environment(\.dismiss) private var dismiss
    
    var body: some View {
        NavigationView {
            VStack {
                // Device filter selector
                if availableDevices.count > 1 {
                    ScrollView(.horizontal, showsIndicators: false) {
                        HStack(spacing: 8) {
                            Button(action: {
                                selectedDeviceFilter = nil
                                refreshLogDates()
                            }) {
                                Text("All Devices")
                                    .padding(.horizontal, 12)
                                    .padding(.vertical, 6)
                                    .background((selectedDeviceFilter == nil) ? Color.blue : Color.gray.opacity(0.2))
                                    .cornerRadius(16)
                                    .foregroundColor((selectedDeviceFilter == nil) ? .white : .primary)
                            }
                            
                            ForEach(availableDevices, id: \.self) { deviceID in
                                Button(action: {
                                    selectedDeviceFilter = deviceID
                                    refreshLogDates()
                                }) {
                                    Text("Device \(deviceID)")
                                        .padding(.horizontal, 12)
                                        .padding(.vertical, 6)
                                        .background((selectedDeviceFilter == deviceID) ? Color.blue : Color.gray.opacity(0.2))
                                        .cornerRadius(16)
                                        .foregroundColor((selectedDeviceFilter == deviceID) ? .white : .primary)
                                }
                            }
                        }
                        .padding(.horizontal)
                    }
                    .padding(.vertical, 8)
                    .background(Color(UIColor.secondarySystemBackground))
                }
                
                ZStack {
                    List {
                        if logDates.isEmpty {
                            Text("No telemetry data recorded")
                                .foregroundColor(.gray)
                        } else {
                            ForEach(logDates, id: \.self) { date in
                                Button(action: {
                                    loadRecordsForDate(date)
                                }) {
                                    HStack {
                                        Image(systemName: "calendar")
                                        Text(dateFormatter.string(from: date))
                                    }
                                }
                                .swipeActions(edge: .trailing) {
                                    Button(role: .destructive) {
                                        dateToDelete = date
                                        showingDeleteAlert = true
                                    } label: {
                                        Label("Delete", systemImage: "trash")
                                    }
                                }
                            }
                        }
                    }
                    
                    if isLoading {
                        ProgressView("Loading records...")
                            .padding()
                            .background(Color(UIColor.systemBackground).opacity(0.8))
                            .cornerRadius(8)
                    }
                }
            }
            .navigationTitle("Telemetry History")
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") {
                        dismiss()
                    }
                }
                
                ToolbarItem(placement: .navigationBarLeading) {
                    Button(action: {
                        refreshLogDates()
                    }) {
                        Label("Refresh", systemImage: "arrow.clockwise")
                    }
                }
            }
            .sheet(isPresented: $showingDataDetail) {
                if let date = selectedDate {
                    TelemetryLogDetailView(
                        date: date, 
                        records: recordsForDate,
                        deviceFilter: selectedDeviceFilter
                    )
                }
            }
            .alert("Delete Records", isPresented: $showingDeleteAlert, presenting: dateToDelete) { date in
                Button("Cancel", role: .cancel) {}
                Button("Delete", role: .destructive) {
                    performDelete(for: date)
                }
            } message: { date in
                let deviceText = selectedDeviceFilter != nil ? " for Device \(selectedDeviceFilter!)" : ""
                Text("Are you sure you want to delete all telemetry records\(deviceText) for \(dateFormatter.string(from: date))?")
            }
            .onAppear {
                loadAvailableDevices()
                refreshLogDates()
            }
        }
    }
    
    private func loadAvailableDevices() {
        DispatchQueue.global(qos: .userInitiated).async {
            let devices = presenter.getAllDeviceIDs()
            
            DispatchQueue.main.async {
                self.availableDevices = devices
            }
        }
    }

    private func performDelete(for date: Date) {
        isLoading = true
        
        // Delete in background to avoid UI freezing
        DispatchQueue.global(qos: .userInitiated).async {
            self.presenter.deleteRecordsForDate(date, deviceID: selectedDeviceFilter)
            
            // Refresh the list on the main thread
            DispatchQueue.main.async {
                self.refreshLogDates()
            }
        }
    }
    
    private func loadRecordsForDate(_ date: Date) {
        isLoading = true
        
        // Use DispatchQueue to avoid UI freezing when loading large datasets
        DispatchQueue.global(qos: .userInitiated).async {
            let calendar = Calendar.current
            let startOfDay = calendar.startOfDay(for: date)
            let endOfDay = calendar.date(byAdding: .day, value: 1, to: startOfDay)!
            
            let records = presenter.getTelemetryRecords(
                deviceID: selectedDeviceFilter, 
                from: startOfDay, 
                to: endOfDay
            )
            
            // Return to main thread for UI updates
            DispatchQueue.main.async {
                recordsForDate = records
                selectedDate = date
                isLoading = false
                showingDataDetail = true
            }
        }
    }
    
    private func refreshLogDates() {
        isLoading = true
        
        DispatchQueue.global(qos: .userInitiated).async {
            let dates = presenter.getAvailableDates(forDeviceID: selectedDeviceFilter)
            
            DispatchQueue.main.async {
                logDates = dates
                isLoading = false
            }
        }
    }
    
    private var dateFormatter: DateFormatter {
        let formatter = DateFormatter()
        formatter.dateStyle = .medium
        return formatter
    }
}

struct TelemetryLogDetailView: View {
    let date: Date
    let records: [TelemetryRecord]
    var deviceFilter: UInt32?
    @Environment(\.dismiss) private var dismiss
    @State private var selectedSegment = 0
    @State private var deviceRecords: [UInt32: [TelemetryRecord]] = [:]
    @State private var devices: [UInt32] = []
    @State private var isExporting = false
    @State private var csvURL: URL?
    
    var body: some View {
        NavigationView {
            VStack {
                // Data summary header
                VStack(alignment: .leading, spacing: 4) {
                    Text("\(records.count) Records")
                        .font(.headline)
                                    
                    if let firstRecord = records.first,
                    let firstTimestamp = firstRecord.timestamp,
                    let lastRecord = records.last,
                    let lastTimestamp = lastRecord.timestamp {
                        
                        Text("From: \(timeFormatter.string(from: firstTimestamp))")
                            .font(.subheadline)
                        Text("To: \(timeFormatter.string(from: lastTimestamp))")
                            .font(.subheadline)
                    }
            
                    if let deviceFilter = deviceFilter {
                        Text("Filtered by Device \(deviceFilter)")
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                    } else {
                        Text("Showing records for all devices")
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                    }
                }
                .frame(maxWidth: .infinity, alignment: .leading)
                .padding(.horizontal)
                .padding(.top, 8)
                
                // Device tabs if we have multiple devices and no filter
                if deviceFilter == nil && devices.count > 1 {
                    ScrollView(.horizontal, showsIndicators: false) {
                        HStack(spacing: 8) {
                            Button(action: {
                                selectedSegment = 0
                            }) {
                                Text("All")
                                    .padding(.horizontal, 12)
                                    .padding(.vertical, 6)
                                    .background(selectedSegment == 0 ? Color.blue : Color.gray.opacity(0.2))
                                    .cornerRadius(16)
                                    .foregroundColor(selectedSegment == 0 ? .white : .primary)
                            }
                            
                            ForEach(Array(devices.enumerated()), id: \.element) { index, device in
                                Button(action: {
                                    selectedSegment = index + 1
                                }) {
                                    Text("Device \(device)")
                                        .padding(.horizontal, 12)
                                        .padding(.vertical, 6)
                                        .background(selectedSegment == index + 1 ? Color.blue : Color.gray.opacity(0.2))
                                        .cornerRadius(16)
                                        .foregroundColor(selectedSegment == index + 1 ? .white : .primary)
                                }
                            }
                        }
                        .padding(.horizontal)
                    }
                    .padding(.vertical, 8)
                }
                
                Divider()
                
                // Record list
                List {
                    // Show records based on selected segment/tab
                    if selectedSegment == 0 || devices.count <= 1 {
                        // Show all records
                        ForEach(0..<records.count, id: \.self) { index in
                            recordView(for: records[index])
                        }
                    } else if selectedSegment > 0 && selectedSegment <= devices.count {
                        // Show records for the selected device
                        let deviceID = devices[selectedSegment - 1]
                        if let deviceRecords = deviceRecords[deviceID] {
                            ForEach(0..<deviceRecords.count, id: \.self) { index in
                                recordView(for: deviceRecords[index])
                            }
                        }
                    }
                }
            }
            .navigationTitle(dateFormatter.string(from: date))
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Close") {
                        dismiss()
                    }
                }
                ToolbarItem(placement: .navigationBarLeading) {
                    Button(action: {
                        // Export file first
                        _ = exportToCSV()
                    }) {
                        Label("Export CSV", systemImage: "square.and.arrow.up")
                    }
                }
            }
            // .sheet(isPresented: $isExporting) {
            //     if let url = self.csvURL {
            //         DocumentPicker(fileURL: url)
            //     } else {
            //         Text("No file to export")
            //     }
            // }
            .onAppear {
                organizeRecordsByDevice()
            }
        }
    }

    private func exportToCSV() -> URL? {
        // Create CSV content
        var csvString = "Device ID,Message #,Time Since Boot,Timestamp," +
            "ADXL Accel X,ADXL Accel Y,ADXL Accel Z," +
            "ISM Primary Accel X,ISM Primary Accel Y,ISM Primary Accel Z," +
            "ISM Primary Gyro X,ISM Primary Gyro Y,ISM Primary Gyro Z," +
            "ISM Secondary Accel X,ISM Secondary Accel Y,ISM Secondary Accel Z," +
            "ISM Secondary Gyro X,ISM Secondary Gyro Y,ISM Secondary Gyro Z," +
            "LSM Accel X,LSM Accel Y,LSM Accel Z," +
            "LSM Gyro X,LSM Gyro Y,LSM Gyro Z," +
            "Barometer Alt,GPS Alt,GPS Fix,GPS Lat,GPS Lon,GPS Satellites," +
            "GPS UTC Day,GPS UTC Hour,GPS UTC ITOW,GPS UTC Min,GPS UTC Month," +
            "GPS UTC Nanos,GPS UTC Sec,GPS UTC Time Accuracy,GPS UTC Valid,GPS UTC Year\n"

        for record in records {
        let timestamp = record.timestamp ?? Date()
        let formattedDate = ISO8601DateFormatter().string(from: timestamp)
        let gpsFix = record.gps_fix ?? "Unknown"
        
        // Format each line with all fields
        let csvLine = """
            \(record.deviceID),\
            \(record.msgNum),\
            \(record.timeSinceBoot),\
            \(formattedDate),\
            \(record.adxl_accel_x),\
            \(record.adxl_accel_y),\
            \(record.adxl_accel_z),\
            \(record.ism_primary_accel_x),\
            \(record.ism_primary_accel_y),\
            \(record.ism_primary_accel_z),\
            \(record.ism_primary_gyro_x),\
            \(record.ism_primary_gyro_y),\
            \(record.ism_primary_gyro_z),\
            \(record.ism_secondary_accel_x),\
            \(record.ism_secondary_accel_y),\
            \(record.ism_secondary_accel_z),\
            \(record.ism_secondary_gyro_x),\
            \(record.ism_secondary_gyro_y),\
            \(record.ism_secondary_gyro_z),\
            \(record.lsm_accel_x),\
            \(record.lsm_accel_y),\
            \(record.lsm_accel_z),\
            \(record.lsm_gyro_x),\
            \(record.lsm_gyro_y),\
            \(record.lsm_gyro_z),\
            \(record.baro_alt),\
            \(record.gps_alt),\
            \"\(gpsFix)\",\
            \(record.gps_lat),\
            \(record.gps_lon),\
            \(record.gps_num_sats),\
            \(record.gps_utc_day),\
            \(record.gps_utc_hour),\
            \(record.gps_utc_itow),\
            \(record.gps_utc_min),\
            \(record.gps_utc_month),\
            \(record.gps_utc_nanos),\
            \(record.gps_utc_sec),\
            \(record.gps_utc_time_accuracy_estimate_ns),\
            \(record.gps_utc_valid),\
            \(record.gps_utc_year)
            """
            
            csvString.append(csvLine + "\n")
        }

        let docsDir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!

        // Create the file URL
        let timestamp = Int(Date().timeIntervalSince1970)
        let fileName = "RocketTelemetry-\(timestamp).csv"
        let fileURL = docsDir.appendingPathComponent(fileName)
        
        print("Trying to write CSV to: \(fileURL.path)")

        // Write to file
        do {
            try csvString.write(to: fileURL, atomically: true, encoding: .utf8)
            print("CSV file created at: \(fileURL)")

            // DispatchQueue.main.async {
            //     // Create and present the document picker directly
            //     let picker = UIDocumentPickerViewController(forExporting: [fileURL])
            //     picker.allowsMultipleSelection = false
                
            //     // Get the current UIViewController to present from
            //     if let windowScene = UIApplication.shared.connectedScenes.first as? UIWindowScene,
            //     let rootVC = windowScene.windows.first?.rootViewController {
            //         // Find the top-most presented controller
            //         var topController = rootVC
            //         while let presentedController = topController.presentedViewController {
            //             topController = presentedController
            //         }
                    
            //         print("Presenting document picker directly")
            //         topController.present(picker, animated: true)
            //     } else {
            //         print("Failed to find a view controller to present from")
            //     }
            // }

            return fileURL
            
        } catch {
            print("Error writing CSV file: \(error)")
            return nil
        }
    }
    
    private func organizeRecordsByDevice() {
        var tempDeviceRecords: [UInt32: [TelemetryRecord]] = [:]
        var uniqueDevices: Set<UInt32> = []
        
        for record in records {
            // Change from using KVC to direct property access
            let deviceID = record.deviceID
            let deviceIDUInt32 = UInt32(deviceID)
            uniqueDevices.insert(deviceIDUInt32)
            
            if tempDeviceRecords[deviceIDUInt32] == nil {
                tempDeviceRecords[deviceIDUInt32] = []
            }
            
            tempDeviceRecords[deviceIDUInt32]?.append(record)
        }
        
        self.deviceRecords = tempDeviceRecords
        self.devices = Array(uniqueDevices).sorted()
    }
    
    @ViewBuilder
    private func recordView(for record: TelemetryRecord) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                // Show device ID badge
                Text("Device \(record.deviceID)")
                    .font(.caption)
                    .padding(.horizontal, 6)
                    .padding(.vertical, 2)
                    .background(Color.blue.opacity(0.2))
                    .cornerRadius(4)
                
                Text("MSG #\(record.msgNum)")
                    .font(.headline)
                Spacer()
                if let timestamp = record.timestamp {
                    Text(timeFormatter.string(from: timestamp))
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            
            HStack(spacing: 8) {
                VStack(alignment: .leading, spacing: 4) {
                    Text("Location:")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    Text("\(String(format: "%.6f", record.gps_lat)), \(String(format: "%.6f", record.gps_lon))")
                        .font(.caption2)
                }
                
                Spacer()
                
                VStack(alignment: .trailing, spacing: 4) {
                    Text("Altitude:")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    Text("\(String(format: "%.1f", record.gps_alt)) m")
                        .font(.caption2)
                }
            }
            
            HStack {
                Label("\(record.gps_num_sats) satellites", systemImage: "antenna.radiowaves.left.and.right")
                    .font(.caption)
                    .foregroundColor(.secondary)
                Spacer()
                Text(record.gps_fix ?? "Unknown Fix")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
        }
        .padding(.vertical, 4)
    }
    
    private var dateFormatter: DateFormatter {
        let formatter = DateFormatter()
        formatter.dateStyle = .medium
        return formatter
    }
    
    private var timeFormatter: DateFormatter {
        let formatter = DateFormatter()
        formatter.timeStyle = .medium
        return formatter
    }
}

// Extract individual device telemetry view for reuse
struct DeviceTelemetryView: View {
    let telemetry: TelemetryData
    
    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            GroupBox(label: Label("Device \(telemetry.deviceID)", systemImage: "antenna.radiowaves.left.and.right")) {
                Text("Device ID: \(telemetry.deviceID)")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
            }
            
            GroupBox(label: Label("GPS Data", systemImage: "location.fill")) {
                VStack(alignment: .leading, spacing: 4) {
                    DataRow(label: "Latitude", value: String(format: "%.6f°", telemetry.gps.lat))
                    DataRow(label: "Longitude", value: String(format: "%.6f°", telemetry.gps.lon))
                    DataRow(label: "Altitude", value: String(format: "%.1f m", telemetry.gps.alt))
                    DataRow(label: "Satellites", value: "\(telemetry.gps.num_sats)")
                    DataRow(label: "GPS Fix", value: telemetry.gps.fix)
                    DataRow(label: "Baro Alt", value: String(format: "%.1f m", telemetry.barometer.altitude))
                }
                .padding(.vertical, 6)
            }
            
            GroupBox(label: Label("System", systemImage: "clock")) {
                VStack(alignment: .leading, spacing: 4) {
                    DataRow(label: "Boot Time", value: "\(telemetry.time_since_boot) ms")
                    DataRow(label: "Message #", value: "\(telemetry.msg_num)")
                    DataRow(label: "Date", value: "\(telemetry.gps.time.day)/\(telemetry.gps.time.month)/\(telemetry.gps.time.year)")
                    DataRow(label: "Time", value: String(format: "%02d:%02d:%02d",
                                                         telemetry.gps.time.hour,
                                                         telemetry.gps.time.min,
                                                         telemetry.gps.time.sec))
                }
                .padding(.vertical, 6)
            }
        }
    }
}

struct DocumentPicker: UIViewControllerRepresentable {
    var fileURL: URL
    @Environment(\.presentationMode) private var presentationMode
    
    func makeUIViewController(context: Context) -> UIDocumentPickerViewController {
        // Create a document picker for exporting the file
        let picker = UIDocumentPickerViewController(forExporting: [fileURL])
        picker.delegate = context.coordinator
        return picker
    }
    
    func updateUIViewController(_ uiViewController: UIDocumentPickerViewController, context: Context) {}
    
    func makeCoordinator() -> Coordinator {
        Coordinator(self)
    }
    
    class Coordinator: NSObject, UIDocumentPickerDelegate {
        let parent: DocumentPicker
        
        init(_ parent: DocumentPicker) {
            self.parent = parent
        }
        
        func documentPicker(_ controller: UIDocumentPickerViewController, didPickDocumentsAt urls: [URL]) {
            // File was successfully exported
            print("Document exported to: \(urls)")
            parent.presentationMode.wrappedValue.dismiss()
        }
        
        func documentPickerWasCancelled(_ controller: UIDocumentPickerViewController) {
            // User cancelled the operation
            print("Document export cancelled")
            parent.presentationMode.wrappedValue.dismiss()
        }
    }
}

#Preview {
    let bluetoothService = BluetoothService()
    let locationService = LocationService()
    let presenter = MainPresenter(bluetoothService: bluetoothService, locationService: locationService)
    return TelemetryDataView(presenter: presenter)
}
