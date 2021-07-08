using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Trajectories.Ui
{
    public class MainViewModel
    {
        public TrajectoriesViewModel P2PFrom { get; }

        public MainViewModel()
        {
            P2PFrom = new TrajectoriesViewModel();
        }
    }
}
