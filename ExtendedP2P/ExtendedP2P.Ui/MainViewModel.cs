using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ExtendedP2P.Ui
{
    public class MainViewModel
    {
        public P2PFromViewModel P2PFrom { get; }

        public MainViewModel()
        {
            P2PFrom = new P2PFromViewModel();
        }
    }
}
