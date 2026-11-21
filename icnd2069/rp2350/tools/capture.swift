import AppKit
import CoreGraphics
func script(_ text: String) -> String {
    var error: NSDictionary?
    let value = NSAppleScript(source: text)!.executeAndReturnError(&error)
    if let error = error {
        fputs("DSView AppleScript error: \(error)\n", stderr)
        exit(1)
    }
    return value.stringValue ?? ""
}
let app = NSRunningApplication.runningApplications(withBundleIdentifier: "com.yourcompany.DSView").first!
app.activate(options: [.activateAllWindows])
Thread.sleep(forTimeInterval: 0.3)
for down in [true, false] {
    CGEvent(keyboardEventSource:nil, virtualKey:53, keyDown:down)!.post(tap:.cghidEventTap)
}
// User's live-debug budget is 50 ms; never silently increase it.
var duration = script("tell application \"System Events\" to tell process \"DSView\" to get name of pop up button 2 of window 1")
if duration == "200.00 ms" || duration == "100.00 ms" {
    let steps = duration == "200.00 ms" ? 2 : 1
    _ = script("tell application \"System Events\" to tell process \"DSView\" to click pop up button 2 of window 1")
    Thread.sleep(forTimeInterval: 0.2)
    for code: UInt16 in Array(repeating: 125, count: steps) + [36] {
        CGEvent(keyboardEventSource:nil, virtualKey:code, keyDown:true)!.post(tap:.cghidEventTap)
        CGEvent(keyboardEventSource:nil, virtualKey:code, keyDown:false)!.post(tap:.cghidEventTap)
        Thread.sleep(forTimeInterval: 0.1)
    }
    duration = script("tell application \"System Events\" to tell process \"DSView\" to get name of pop up button 2 of window 1")
}
guard duration == "50.00 ms" else {
    fputs("Expected 50.00 ms; DSView currently selects \(duration)\n", stderr)
    exit(1)
}
print(duration)
if !CommandLine.arguments.contains("--save-only") {
    _ = script("tell application \"System Events\" to tell process \"DSView\" to click button \"立即(I)\" of window 1")
    Thread.sleep(forTimeInterval: 5)
}
app.activate(options: [.activateAllWindows])
Thread.sleep(forTimeInterval: 0.3)
for down in [true, false] {
    CGEvent(keyboardEventSource:nil, virtualKey:53, keyDown:down)!.post(tap:.cghidEventTap)
}
Thread.sleep(forTimeInterval: 0.2)
let origin = script("""
tell application "System Events" to tell process "DSView"
    set p to position of window 1
    return (item 1 of p as text) & "," & (item 2 of p as text)
end tell
""")
    .split(separator: ",").compactMap { Double($0.trimmingCharacters(in: .whitespaces)) }
guard origin.count == 2 else { exit(1) }
// Verified for DSView 1.3.2, 1281x703 window; relative to its origin.
for point in [CGPoint(x:origin[0]+956,y:origin[1]+77), CGPoint(x:origin[0]+973,y:origin[1]+173)] {
    CGEvent(mouseEventSource:nil, mouseType:.leftMouseDown, mouseCursorPosition:point, mouseButton:.left)!.post(tap:.cghidEventTap)
    CGEvent(mouseEventSource:nil, mouseType:.leftMouseUp, mouseCursorPosition:point, mouseButton:.left)!.post(tap:.cghidEventTap)
    Thread.sleep(forTimeInterval:0.5)
}
let path = script("tell application \"System Events\" to tell process \"DSView\" to get value of text area 1 of window 1")
guard !FileManager.default.fileExists(atPath:path) else { fatalError("Capture already exists: \(path)") }
_ = script("tell application \"System Events\" to tell process \"DSView\" to click button \"确定\" of group 1 of window 1")
Thread.sleep(forTimeInterval:2)
guard FileManager.default.fileExists(atPath:path) else {
    fputs("DSView did not create capture: \(path)\n", stderr)
    exit(1)
}
print("SAVED_PATH=\(path)")
