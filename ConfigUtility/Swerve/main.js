const { app, BrowserWindow, Tray, Menu, nativeImage } = require('electron')
// include the Node.js 'path' module at the top of your file
const path = require('node:path')
const isMac = process.platform === 'darwin'
let tray

const createWindow = () => {
  const win = new BrowserWindow({
    width: 800,
    height: 600,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      nodeIntegration: true
    },
  })

  win.loadFile('index.html')
}
app.whenReady().then(() => {
  createWindow()

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow()
  })
})

app.on('window-all-closed', () => {

  if (process.platform !== 'darwin') app.quit()
})