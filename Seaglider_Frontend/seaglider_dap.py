# Seaglider DAP .exe
#initial version A_0.1 
# OP w/ significant help from gemini AI
#HH provided psuedo code that greatly assisted in setting up framework

# %% Import Libraries

#KEY NOTE: Download and install Matlab Runtime Compiler SDK R2024b to not need
#to use a seperate version of matlab
#this also means you do not need to pay for matlab if your organization does
#not cover it
#if you choose to transfer the .m files into python you can uninstall the 
#matlab compiler SDK to save time and storage.
import tkinter as tk
from tkinter import filedialog, messagebox
import os
#threading allows us to load matlab without timing out
import threading
import sys
import webbrowser

# %% Verifies that Matlab runtime R2024b is installed 
def verify_matlab_runtime(existing_root):
    """
    Scans the MATLAB directory to find either the R2024b Full Install 
    or the v242 Runtime.
    """
    found = False
    runtime_url = "https://www.mathworks.com/products/compiler/matlab-runtime.html"
    
    # Define common locations
    program_files = os.environ.get("ProgramFiles", "C:\\Program Files")
    matlab_root = os.path.join(program_files, "MATLAB")

    # 1. Check if the MATLAB folder even exists
    if os.path.exists(matlab_root):
        # Walk through the MATLAB folder to find 2024b/v242
        # This handles nested folders like 'MATLAB Runtime/v242'
        for root, dirs, files in os.walk(matlab_root):
            # We look for the folder names that indicate 2024b
            if "v242" in dirs or "R2024b" in dirs:
                found = True
                break
    
    # 2. Backup check: Is it in the system PATH?
    if not found:
        path_env = os.environ.get("PATH", "").lower()
        if "v242" in path_env or "r2024b" in path_env:
            found = True

    # 3. If still not found, show the error
    if not found:
        error_msg = (
            "MATLAB R2024b (24.2) Not Detected!\n\n"
            "We found the MATLAB folder, but not the specific 2024b Runtime.\n\n"
            "Please ensure you have installed 'MATLAB Runtime R2024b'.\n"
            "Would you like to download it now?"
        )
        
        existing_root.attributes("-topmost", True) 
        user_choice = messagebox.askyesno("Missing Dependency", error_msg, parent=existing_root)
        
        if user_choice:
            webbrowser.open(runtime_url)
        
        sys.exit(1)
    else:
        print("MATLAB 2024b environment detected.")
        
# %% Python frontend
class SeagliderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SEAGLIDER D.A.P. System (Standalone)")
        self.root.geometry("550x500")
        
        self.log_filepath = None
        self.nc_filepath = None
        self.ml_pkg = None # This will hold our compiled MATLAB instance

        # ui setup
        self.setup_ui()

#Buttons that select files and show if no files selected
    def setup_ui(self):
       
        tk.Label(self.root, text="Seaglider Fault Diagnoser", font=("Helvetica", 14, "bold")).pack(pady=10)
        self.btn_log = tk.Button(self.root, text="1. Select LOG File", command=self.select_log, width=30)
        self.btn_log.pack(pady=5)
        self.lbl_log = tk.Label(self.root, text="No LOG file selected", fg="red")
        self.lbl_log.pack()
        self.btn_nc = tk.Button(self.root, text="2. Select NC/ENG File", command=self.select_nc, width=30)
        self.btn_nc.pack(pady=5)
        self.lbl_nc = tk.Label(self.root, text="No NC file selected", fg="red")
        self.lbl_nc.pack()
        self.btn_run = tk.Button(self.root, text="3. RUN DIAGNOSIS", command=self.start_pipeline_thread, width=30, bg="green", fg="white", font=("Helvetica", 10, "bold"))
        self.btn_run.pack(pady=20)
        self.txt_output = tk.Text(self.root, height=12, width=60, state=tk.DISABLED, bg="#f4f4f4")
        self.txt_output.pack(pady=5)

#file path device for the buttons above add more as you add buttons
    def select_log(self):
        filepath = filedialog.askopenfilename(filetypes=[("LOG Files", "*.log")])
        if filepath:
            self.log_filepath = filepath
            self.lbl_log.config(text=os.path.basename(filepath), fg="green")

    def select_nc(self):
        filepath = filedialog.askopenfilename(filetypes=[("NC Files", "*.nc")])
        if filepath:
            self.nc_filepath = filepath
            self.lbl_nc.config(text=os.path.basename(filepath), fg="green")

#shows up if you're missing any files
    def start_pipeline_thread(self):
        if not self.log_filepath or not self.nc_filepath:
            messagebox.showwarning("Missing Files", "Please select files first.")
            return
        self.btn_run.config(state=tk.DISABLED, bg="grey")
        threading.Thread(target=self.run_compiled_pipeline).start()

#runs matlab runtime (gross pls convert to python, i have an aversion to paying money to use matlab)
    def run_compiled_pipeline(self):
        try:
            self.print_to_output("Initializing MATLAB Runtime (this can take up to 30 seconds on the first run)...")
            
            # imports compiled package
            try:
                self.print_to_output("Initializing MATLAB Runtime... (Wait ~30s)\n")
                import SeagliderLib
                if self.ml_pkg is None:
                    self.ml_pkg = SeagliderLib.initialize()
            except ImportError:
                self.print_to_output("ERROR: SeagliderLib not installed. Run 'python setup.py install' in the redist folder.\n")
                return

            # initializes matlab library ie(start_matlab ish)
            if self.ml_pkg is None:
                self.ml_pkg = SeagliderLib.initialize()

            self.print_to_output("Runtime Loaded. Unpacking files...\n")
            
            # call functions (using self. instead of eng)
            # The SDK uses standard Python arguments (another reminder to just switch to python please)
            log_matrix = self.ml_pkg.Log_File_Unpacker(self.log_filepath)
            nc_matrix = self.ml_pkg.NC_File_Unpacker(self.nc_filepath)
            
            self.print_to_output("Loading Parameters & Running Dynamics...\n")
            
            # returns coefficients as a tuple in python
            #using wrapper function to get params/coefficients
            params, coefs = self.ml_pkg.load_constants(nargout=2)

            # creating the dynamics matrix
            nom_sim_matrix = self.ml_pkg.Create_Dynamics_Matrix(log_matrix, nc_matrix, coefs, params)

            self.print_to_output("Running Diagnoser...\n")

            # runs the diagnoser
            diagnose = self.ml_pkg.missioncompare(nom_sim_matrix, nc_matrix)

            #shows the resulsts
            self.print_to_output(f"\n--- DIAGNOSIS COMPLETE ---\n{diagnose}\n")

        except Exception as e:
            self.print_to_output(f"\nERROR:\n{str(e)}\n")
        
        finally:
            self.btn_run.config(state=tk.NORMAL, bg="green")

    def print_to_output(self, message):
        self.txt_output.config(state=tk.NORMAL)
        self.txt_output.insert(tk.END, message)
        self.txt_output.see(tk.END)
        self.txt_output.config(state=tk.DISABLED)

# %%
if __name__ == "__main__":
    root = tk.Tk()
    
    # 1. Hide the window while we do the quick folder check
    root.withdraw() 
    
    print("Checking system...")
    verify_matlab_runtime(root) 
    print("Check passed!")

    # 2. Re-show the window immediately
    root.deiconify() 
    
    # 3. Initialize the App
    app = SeagliderApp(root)
    
    # 4. Start the loop
    # The GUI will now appear. The "heavy" MATLAB loading 
    # will only happen when the user clicks 'Run'.
    root.mainloop()