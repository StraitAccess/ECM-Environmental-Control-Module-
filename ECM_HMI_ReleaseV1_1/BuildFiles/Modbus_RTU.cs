//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.42000
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

namespace Neo.ApplicationFramework.Generated
{
	using Neo.ApplicationFramework.Tools.Actions;
	
	
	public partial class Modbus_RTU : Neo.ApplicationFramework.Tools.OpcClient.DataSourceContainer
	{
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem3;
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem1;
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem2;
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem4;
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem5;
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem6;
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem10;
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem8;
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem9;
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem11;
		
		private Neo.ApplicationFramework.Tools.OpcClient.DataItem DataItem7;
		
		public Modbus_RTU()
		{
			this.InitializeComponent();
			this.ApplyLanguageInternal();
		}
		
		private void InitializeComponent()
		{
			this.InitializeObjectCreations();
			this.InitializeBeginInits();
			this.InitializeObjects();
			this.InitializeEndInits();
			this.ConnectDataBindings();
		}
		
		protected override void Dispose(bool disposing)
		{
			base.Dispose(disposing);
		}
		
		[System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
		public override void ConnectDataBindings()
		{
			base.ConnectDataBindings();
		}
		
		private void InitializeObjectCreations()
		{
			this.DataItem3 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem3", "5:40000", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 1D, ((short)(1)), false);
			this.DataItem1 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem1", "5:40001", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 1D, ((short)(1)), false);
			this.DataItem2 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem2", "5:40002", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 1D, ((short)(1)), false);
			this.DataItem4 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem4", "5:40003", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 0.1D, ((short)(1)), false);
			this.DataItem5 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem5", "5:40004", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 1D, ((short)(5)), false);
			this.DataItem6 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem6", "5:40005", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 0.1D, ((short)(1)), false);
			this.DataItem10 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem10", "5:40009", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 1D, ((short)(1)), false);
			this.DataItem8 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem8", "5:40007", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 1D, ((short)(1)), false);
			this.DataItem9 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem9", "5:40008", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 1D, ((short)(1)), false);
			this.DataItem11 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem11", "5:40010", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 1D, ((short)(1)), false);
			this.DataItem7 = new Neo.ApplicationFramework.Tools.OpcClient.DataItem("DataItem7", "5:40006", ((Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE)(Neo.ApplicationFramework.Interop.DataSource.BEDATATYPE.DT_INTEGER2)), ((short)(1)), 0D, 1D, ((short)(1)), false);
		}
		
		private void InitializeBeginInits()
		{
			((System.ComponentModel.ISupportInitialize)(this)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem3)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem1)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem2)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem4)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem5)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem6)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem10)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem8)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem9)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem11)).BeginInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem7)).BeginInit();
		}
		
		private void InitializeEndInits()
		{
			((System.ComponentModel.ISupportInitialize)(this.DataItem3)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem1)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem2)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem4)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem5)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem6)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem10)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem8)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem9)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem11)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this.DataItem7)).EndInit();
			((System.ComponentModel.ISupportInitialize)(this)).EndInit();
		}
		
		private void InitializeObjects()
		{
			this.DataItems.Add(this.DataItem3);
			this.DataItems.Add(this.DataItem1);
			this.DataItems.Add(this.DataItem2);
			this.DataItems.Add(this.DataItem4);
			this.DataItems.Add(this.DataItem5);
			this.DataItems.Add(this.DataItem6);
			this.DataItems.Add(this.DataItem10);
			this.DataItems.Add(this.DataItem8);
			this.DataItems.Add(this.DataItem9);
			this.DataItems.Add(this.DataItem11);
			this.DataItems.Add(this.DataItem7);
			this.Driver = "Modicon_Modbus_RTU_ASCII_Pre2.Modicon_Modbus_RTU_ASCII_Pre2.1";
			this.Node = "";
			this.Servername = "Beijer.InprocessNeo.1";
			this.TargetInfoString = "iXDeveloper;WinCe6;Beijer;Exter iX;X2 base 5;en;";
			this.Url = "";
		}
		
		[System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
		private void ApplyLanguageInternal()
		{
			Neo.ApplicationFramework.Tools.MultiLanguage.MultiLanguageResourceManager resources = new Neo.ApplicationFramework.Tools.MultiLanguage.MultiLanguageResourceManager(typeof(Modbus_RTU));
		}
		
		[System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
		protected override void ApplyLanguage()
		{
			this.ApplyLanguageInternal();
			base.ApplyLanguage();
		}
	}
}
